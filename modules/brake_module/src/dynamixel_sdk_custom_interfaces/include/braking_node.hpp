// This scirpt based on Dynamixel_SDK_examples and writhen by Orel Hamamy Almog


#ifndef READ_WRITE_NODE_HPP_
#define READ_WRITE_NODE_HPP_

#include <cstdio>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"
#include "dynamixel_sdk/dynamixel_sdk.h"
#include "dynamixel_sdk_custom_interfaces/msg/set_brake.hpp"
#include "dynamixel_sdk_custom_interfaces/srv/get_position.hpp"


class BrakingNode : public rclcpp::Node
{
public:
  using SetBrake = dynamixel_sdk_custom_interfaces::msg::SetBrake;
  using GetPosition = dynamixel_sdk_custom_interfaces::srv::GetPosition;
  void get_present_position(const std::shared_ptr<GetPosition::Request> request,
    std::shared_ptr<GetPosition::Response> response);
  void set_brake(const SetBrake::SharedPtr msg);

  BrakingNode();
  virtual ~BrakingNode();

private:
  rclcpp::Subscription<SetBrake>::SharedPtr set_brake_subscriber_;
  rclcpp::Service<GetPosition>::SharedPtr get_position_server_;
  int init_position;

  int present_position;
};

#endif  // READ_WRITE_NODE_HPP_
