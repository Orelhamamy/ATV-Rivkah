
#include "rclcpp/rclcpp.hpp"
#include "velocity_node.hpp"

#include <stdlib.h>
#include <iostream>
#include "serial/serial.h"


#define NODE_NAME "velocity_node"

VelocityNode::VelocityNode() : Node(NODE_NAME){
    RCLCPP_INFO(this->get_logger(), "Run %s.", NODE_NAME);
    this->declare_parameter<std::string>("vel_usb_port", "/dev/ThrottleTeensy");
    this->declare_parameter<std::string>("vel_topic", "/vel_cmd");
    this->declare_parameter("qos_depth", 10);
    this->declare_parameter("baund_rate", 57600);
    this->declare_parameter("pub_rate_ms", 10);

    this->get_parameter("pub_rate_ms", pubRate_);
    this->get_parameter("vel_topic", topic_);
    this->get_parameter("qos_depth", qosDepth);
    strPortName_ = this->get_parameter("vel_usb_port").get_value<std::string>();
    this->get_parameter("baund_rate", baundRate_);
    portName_ = realpath(strPortName_.c_str(), NULL);
    if (portName_ == NULL)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to find symbolic serial port %s", strPortName_.c_str());
        rclcpp::shutdown();
    }

    // serial::Serial serialPort(portName_, baundRate_, serial::Timeout::simpleTimeout(1));
    // if (!serialPort.isOpen()){
    //     RCLCPP_ERROR(this->get_logger(), "Faild opening port: %s", this->portName_);
    //     rclcpp::shutdown();
    // }
    
    const auto QOS_RKL10V =
        rclcpp::QoS(rclcpp::KeepLast(qosDepth)).reliable().durability_volatile();
    ackermann_msg_subscriber_ = this->create_subscription<Ackermann_msg>(
        topic_,
        QOS_RKL10V,
        std::bind(&VelocityNode::ackermann_cb, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&VelocityNode::checkSerialData, this));
    estimateSpeedPub_ = this->create_publisher<std_msgs::msg::Float32>("est_wheel_speed", qosDepth);

    // Open and configure the serial port
    try {
        serialPort_ = std::make_unique<serial::Serial>(portName_, baundRate_, serial::Timeout::simpleTimeout(1));
        if (serialPort_->isOpen()) {
            RCLCPP_INFO(this->get_logger(), "Serial port %s opened successfully at %d baud.", portName_, baundRate_);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to open serial port %s", portName_);
            rclcpp::shutdown();
        }
    } catch (const serial::IOException& e) {
        RCLCPP_ERROR(this->get_logger(), "IOException: %s", e.what());
    }
}

VelocityNode::~VelocityNode() {
    if (serialPort_ && serialPort_->isOpen()) {
        serialPort_->close();
    }
}


void VelocityNode::ackermann_cb(const Ackermann_msg::SharedPtr msg){
    this->currentSpeedCmd_ = msg->drive.speed;
    RCLCPP_INFO(this->get_logger(), "Get speed command: %.2f", this->currentSpeedCmd_);
    std::string speedCmdStr = std::to_string(this->currentSpeedCmd_) + "\n";
    serialPort_->write(speedCmdStr);
    RCLCPP_DEBUG(this->get_logger(), "Sent to serial: %s.",speedCmdStr.c_str());
}

void VelocityNode::checkSerialData() {
    size_t bytesAvailable = serialPort_->available();
    if (bytesAvailable > 0) {
        std::string receivedData = serialPort_->read(bytesAvailable + 1);
        RCLCPP_DEBUG(this->get_logger(), "Received: %s", receivedData.c_str());
        // RCLCPP_DEBUG(this->get_logger(), "received[0]: %c, received[last]: %ld", receivedData[0], receivedData[receivedData.find('>')]);
        // Convert received data to a float for estimated speed, if possible
        size_t st_inx = receivedData.find('<');
        size_t end_inx = receivedData.find('>');
        if (st_inx!=std::string::npos && end_inx!=std::string::npos) {
            try {
                receivedData = receivedData.substr(st_inx + 1, end_inx - 1);
                estimatedSpeed_ = std::stof(receivedData);
                RCLCPP_DEBUG(this->get_logger(), "Received data as float: %f", estimatedSpeed_);
                
                // Publish the estimated speed
                std_msgs::msg::Float32 message;
                message.data = estimatedSpeed_;
                estimateSpeedPub_->publish(message);
                
            } catch (const std::invalid_argument& e) {
                RCLCPP_ERROR(this->get_logger(), "Failed to convert received data to float: %s", e.what());
            }
        }
    }

    }

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VelocityNode>());
    rclcpp::shutdown();
	// //open arduino device file (linux)
    // std::ofstream arduino;
	// arduino.open("/dev/ttyACM0");
    // float a = 1548.185;
	// //write to it
    // arduino << a;
    // std::cout <<"sending:"<< a << '\n';
	// arduino.close();

	return 0;
}