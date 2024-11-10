#ifndef VELOCITY_NODE
#define VELOCITY_NODE

#include "rclcpp/rclcpp.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"
#include <serial/serial.h>


class VelocityNode : public rclcpp::Node {
    public: 
        using Ackermann_msg = ackermann_msgs::msg::AckermannDriveStamped;

        VelocityNode();
        virtual ~VelocityNode();

    private:
        // ROS communication members    
        rclcpp::Subscription<Ackermann_msg>::SharedPtr ackermann_msg_subscriber_;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr estimateSpeedPub_;
        rclcpp::TimerBase::SharedPtr timer_;

        // Serial communication members
        std::unique_ptr<serial::Serial> serialPort_;


        void checkSerialData();
        void ackermann_cb(const Ackermann_msg::SharedPtr msg);
        int openSerialPort();
        bool configureSerialPort();

        float currentSpeedCmd_, estimatedSpeed_;
        std::string topic_;
        std::string strPortName_;
        const char *portName_;
        int8_t qosDepth = 0;
        int baundRate_, pubRate_, portHandler_;

};  

#endif // VELOCITY_NODE