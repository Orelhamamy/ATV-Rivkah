#include <cstdio>
#include <memory>
#include <string>

#include "dynamixel_sdk/dynamixel_sdk.h"
#include "dynamixel_sdk_custom_interfaces/msg/set_brake.hpp"
#include "dynamixel_sdk_custom_interfaces/srv/get_position.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"

#include "braking_node.hpp"

// Control table address for X series (except XL-320)
#define ADDR_OPERATING_MODE 11
#define ADDR_TORQUE_ENABLE 64
#define ADDR_GOAL_CURRENT 102 // [-38,38]  unit 2.69(mA)
#define ADDR_GOAL_POSITION 116
#define ADDR_PRESENT_POSITION 132

// Protocol version
#define PROTOCOL_VERSION 2.0  // Default Protocol version of DYNAMIXEL X series.

// Default setting
#define BAUDRATE 57600  // Default Baudrate of DYNAMIXEL X series
#define DEVICE_NAME "/dev/rear_brake"  // [Linux]: "/dev/ttyUSB*", [Windows]: "COM*"

#define MAX_CURRRENT 38 // XM540-W270-T max current 2.69(mA) 
// #define MOTOR_DIRECTION -1 //
#define NODE_NAME "braking_node"

dynamixel::PortHandler * portHandler;
dynamixel::PacketHandler * packetHandler;

uint8_t dxl_error = 0;
uint32_t goal_position = 0;
int dxl_comm_result = COMM_TX_FAIL;
std::shared_ptr<dynamixel_sdk_custom_interfaces::srv::GetPosition::Request> request ;
std::shared_ptr<dynamixel_sdk_custom_interfaces::srv::GetPosition::Response> response;

BrakingNode::BrakingNode()
: Node(NODE_NAME)
{
  
  RCLCPP_INFO(this->get_logger(), "Run %s.", NODE_NAME);



  this->declare_parameter("qos_depth", 10);
  int8_t qos_depth = 0;
  this->get_parameter("qos_depth", qos_depth);

  const auto QOS_RKL10V =
    rclcpp::QoS(rclcpp::KeepLast(qos_depth)).reliable().durability_volatile();

  get_position_server_ = create_service<GetPosition>("get_position",
                                                    std::bind(&BrakingNode::get_present_position, this, std::placeholders::_1, std::placeholders::_2));
                                                  
  get_present_position(request, response);
  init_position = response->position;

    set_brake_subscriber_ =
    this->create_subscription<SetBrake>(
    "set_brake",
    QOS_RKL10V,
    std::bind(&BrakingNode::set_brake, this, std::placeholders::_1)
    );

  
}

BrakingNode::~BrakingNode()
{
}

void BrakingNode::get_present_position(
    const std::shared_ptr<GetPosition::Request> request,
    std::shared_ptr<GetPosition::Response> response)
    {
      // Read Present Position (length : 4 bytes) and Convert uint32 -> int32
      // When reading 2 byte data from AX / MX(1.0), use read2ByteTxRx() instead.
      dxl_comm_result = packetHandler->read4ByteTxRx(
        portHandler,
        (uint8_t) request->id,
        ADDR_PRESENT_POSITION,
        reinterpret_cast<uint32_t *>(&present_position),
        &dxl_error
      );

      RCLCPP_INFO(
        this->get_logger(),
        "Get [ID: %d] [Present Position: %d]",
        request->id,
        present_position
      );

      response->position = present_position;
    };

void BrakingNode::set_brake(const SetBrake::SharedPtr msg){
    {
      uint8_t dxl_error = 0;
      request->id = 1;
      // Position Value of X series is 4 byte data.
      // For AX & MX(1.0) use 2 byte data(uint16_t) for the Position Value.
      int16_t goal_brake = msg->brake;  // Convert int32 -> uint16
      if (goal_brake > 10 || goal_brake < 0){
        RCLCPP_INFO(this->get_logger(), "The Brake command should be in the range [0, 10].");
        goal_brake = goal_brake > 10 ? 10 : 0;
        RCLCPP_INFO(this->get_logger(), "Set brake command to %d.", goal_brake);
      }
      if (goal_brake==0){
        RCLCPP_INFO(this->get_logger(), "init_pos %d.", init_position);
        this->get_present_position(request, response);
        while (std::abs(init_position - present_position) > 100) {
          RCLCPP_INFO(this->get_logger(), "present_position %d.", present_position);
          uint32_t sign = (init_position - present_position) > 0 ? 1: -1;
          dxl_comm_result =
            packetHandler->write2ByteTxRx(
            portHandler,
            (uint8_t) msg->id,
            ADDR_GOAL_CURRENT,
            sign * MAX_CURRRENT,
            &dxl_error
            );
          this->get_present_position(request, response);
        }       
        dxl_comm_result =
            packetHandler->write2ByteTxRx(
            portHandler,
            (uint8_t) msg->id,
            ADDR_GOAL_CURRENT,
            0,
            &dxl_error
            );
      }
      else{

      uint16_t goal_current = (unsigned int) (goal_brake * MAX_CURRRENT / 10); // Convert int32 -> uint16
      // Write Goal Position (length : 4 bytes)
      // When writing 2 byte data to AX / MX(1.0), use write2ByteTxRx() instead.
      // RCLCPP_INFO(this->get_logger(), "[Goal Current: %u]", goal_current);
      dxl_comm_result =
      packetHandler->write2ByteTxRx(
        portHandler,
        (uint8_t) msg->id,
        ADDR_GOAL_CURRENT,
        goal_current,
        &dxl_error
      );
      }

      if (dxl_comm_result != COMM_SUCCESS) {
        RCLCPP_INFO(this->get_logger(), "%s", packetHandler->getTxRxResult(dxl_comm_result));
      } else if (dxl_error != 0) {
        RCLCPP_INFO(this->get_logger(), "%s", packetHandler->getRxPacketError(dxl_error));
      } else {
        RCLCPP_INFO(this->get_logger(), "Set [ID: %d] [Goal Brake: %d]", msg->id, goal_brake);
      }
    }
}

void setupDynamixel(uint8_t dxl_id)
{
  // Use Current Control Mode
  dxl_comm_result = packetHandler->write1ByteTxRx(
    portHandler,
    dxl_id,
    ADDR_OPERATING_MODE,
    0,
    &dxl_error
  );

  if (dxl_comm_result != COMM_SUCCESS) {
    RCLCPP_ERROR(rclcpp::get_logger(NODE_NAME), "Failed to set Current Control Mode.");
  } else {
    RCLCPP_INFO(rclcpp::get_logger(NODE_NAME), "Succeeded to set Current Control Mode.");
  }

  // Enable Torque of DYNAMIXEL
  dxl_comm_result = packetHandler->write1ByteTxRx(
    portHandler,
    dxl_id,
    ADDR_TORQUE_ENABLE,
    1,
    &dxl_error
  );

  if (dxl_comm_result != COMM_SUCCESS) {
    RCLCPP_ERROR(rclcpp::get_logger(NODE_NAME), "Failed to enable torque.");
  } else {
    RCLCPP_INFO(rclcpp::get_logger(NODE_NAME), "Succeeded to enable torque.");
  }
}

int main(int argc, char * argv[])
{
  portHandler = dynamixel::PortHandler::getPortHandler(DEVICE_NAME);
  packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
  response = std::make_shared<dynamixel_sdk_custom_interfaces::srv::GetPosition::Response> ();
  request = std::make_shared<dynamixel_sdk_custom_interfaces::srv::GetPosition::Request> ();
  request->id = 1;
  // Open Serial Port
  dxl_comm_result = portHandler->openPort();
  if (dxl_comm_result == false) {
    RCLCPP_ERROR(rclcpp::get_logger(NODE_NAME), "Failed to open the port!");
    return -1;
  } else {
    RCLCPP_INFO(rclcpp::get_logger(NODE_NAME), "Succeeded to open the port.");
  }

  // Set the baudrate of the serial port (use DYNAMIXEL Baudrate)
  dxl_comm_result = portHandler->setBaudRate(BAUDRATE);
  if (dxl_comm_result == false) {
    RCLCPP_ERROR(rclcpp::get_logger(NODE_NAME), "Failed to set the baudrate!");
    return -1;
  } else {
    RCLCPP_INFO(rclcpp::get_logger(NODE_NAME), "Succeeded to set the baudrate.");
  }

  setupDynamixel(BROADCAST_ID);

  rclcpp::init(argc, argv);

  auto brakingnode = std::make_shared<BrakingNode>();
  rclcpp::spin(brakingnode);
  rclcpp::shutdown();

  // Disable Torque of DYNAMIXEL
  packetHandler->write1ByteTxRx(
    portHandler,
    BROADCAST_ID,
    ADDR_TORQUE_ENABLE,
    0,
    &dxl_error
  );

  return 0;
}
