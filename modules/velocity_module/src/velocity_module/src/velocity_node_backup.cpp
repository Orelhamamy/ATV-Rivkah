
#include "rclcpp/rclcpp.hpp"

#include <iostream>
#include <fstream>
#include "velocity_node.hpp"
#include <errno.h>
#include <termios.h>
#include <unistd.h>        // For read(), write(), close()
#include <fcntl.h>         // For open(), O_RDWR
#include <sys/select.h>    // For select()


#define NODE_NAME "velocity_node"

VelocityNode::VelocityNode() : Node(NODE_NAME){
        RCLCPP_INFO(this->get_logger(), "Run %s.", NODE_NAME);
        this->declare_parameter<std::string>("vel_usb_port", "/dev/ttyACM0");
        this->declare_parameter<std::string>("vel_topic", "/vel_cmd");
        this->declare_parameter("qos_depth", 10);
        this->declare_parameter("baund_rate", 57600);
        this->declare_parameter("pub_rate_ms", 10);

        this->get_parameter("pub_rate_ms", pubRate_);
        this->get_parameter("vel_topic", topic_);
        this->get_parameter("qos_depth", qosDepth);
        portName_ = this->get_parameter("vel_usb_port").get_value<std::string>().c_str();
        this->get_parameter("baund_rate", baundRate_);

        if (VelocityNode::openSerialPort()==-1){
            RCLCPP_ERROR(this->get_logger(), "Faild opening port: %s", this->portName_);
            rclcpp::shutdown();
        }

        VelocityNode::configureSerialPort();
        
        const auto QOS_RKL10V =
            rclcpp::QoS(rclcpp::KeepLast(qosDepth)).reliable().durability_volatile();
        ackermann_msg_subscriber_ = this->create_subscription<Ackermann_msg>(
            topic_,
            QOS_RKL10V,
            std::bind(&VelocityNode::ackermann_cb, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&VelocityNode::checkSerialData, this));
        estimateSpeedPub_ = this->create_publisher<std_msgs::msg::String>("serial_data", qosDepth);
}

VelocityNode::~VelocityNode(){ 
    RCLCPP_INFO(this->get_logger(), "Starting %s shutdown.", NODE_NAME);
    if (this->portHandler_ >= 0){
        close(this->portHandler_);
    }
}

// Function to open the serial port
int VelocityNode::openSerialPort(){
    this->portHandler_= open(this->portName_, O_RDWR | O_NOCTTY | O_SYNC);
    if (this->portHandler_ < 0) {
        RCLCPP_ERROR(this->get_logger(), "Error opening: %s: %s.",this->portName_, strerror(errno));
        return -1;
    }
    return this->portHandler_;
}

bool VelocityNode::configureSerialPort() {       
    struct termios tty;
    if (tcgetattr(this->portHandler_, &tty) != 0) {
        RCLCPP_ERROR(this->get_logger(), "Error from tcgetattr: %s", strerror(errno));
        return false;
    }

    cfsetospeed(&tty, this->baundRate_);
    cfsetispeed(&tty, this->baundRate_);

    tty.c_cflag
        = (tty.c_cflag & ~CSIZE) | CS8; // 8-bit characters
    tty.c_iflag &= ~IGNBRK; // disable break processing
    tty.c_lflag = 0; // no signaling chars, no echo, no
                    // canonical processing
    tty.c_oflag = 0; // no remapping, no delays
    tty.c_cc[VMIN] = 0; // read doesn't block
    tty.c_cc[VTIME] = 5; // 0.5 seconds read timeout

    tty.c_iflag &= ~(IXON | IXOFF
                    | IXANY); // shut off xon/xoff ctrl

    tty.c_cflag
        |= (CLOCAL | CREAD); // ignore modem controls,
                            // enable reading
    tty.c_cflag &= ~(PARENB | PARODD); // shut off parity
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr(this->portHandler_, TCSANOW, &tty) != 0) {
        RCLCPP_ERROR(this->get_logger(),"Error from tcsetattr: %s",strerror(errno));
        return false;
    }
    return true;
}

void VelocityNode::ackermann_cb(const Ackermann_msg::SharedPtr msg){
    this->currentSpeedCmd_ = msg->drive.speed;
    RCLCPP_INFO(this->get_logger(), "Get speed command: %f", this->currentSpeedCmd_);
    std::string data = std::to_string(this->currentSpeedCmd_);
    data.append("\r\n");
    RCLCPP_INFO(this->get_logger(), "data: %s",data.c_str());
    int bytesWritten = write(this->portHandler_, data.c_str() , sizeof(data));
    RCLCPP_INFO(this->get_logger(),"bytesWritten: %d",bytesWritten);
    if (bytesWritten < 0) {
        RCLCPP_ERROR(this->get_logger(), "Error writing to serial port: %s", strerror(errno));
    }
}

void VelocityNode::checkSerialData() {
    fd_set read_fds;
    FD_ZERO(&read_fds);
    FD_SET(this->portHandler_, &read_fds);

    struct timeval timeout;
    timeout.tv_sec = 0;
    timeout.tv_usec = 1000; // Set timeout to 1 ms

    int result = select(this->portHandler_ + 1, &read_fds, nullptr, nullptr, &timeout);
    RCLCPP_INFO(this->get_logger(),"result : %d.", result);
    if (result > 0 && FD_ISSET(this->portHandler_, &read_fds)) {
        char buffer[256];
        char newbuffer[256];
        unsigned int b=0;
        int i, j;
        
        int bytesRead = read(this->portHandler_, buffer, sizeof(buffer) - 1);
        if (bytesRead > 0) {
            buffer[bytesRead] = '\0';  // Null-terminate the string
            
            for(i = 0; i<256 && buffer[i] != '<'; i++);
            for(j = i; i<256 && buffer[j] != '>'; j++);
            // RCLCPP_INFO(this->get_logger(),"i : %d.", i);
            
            strncpy(newbuffer, buffer + i + 1, j-i);
            newbuffer[j-i] = '\n';
            memcpy(&b, &buffer[i+1],1);
            memcpy(((int*) ((char*) &b+1)), &buffer[i],1);
            // Publish the data as a ROS 2 message
            auto message = std_msgs::msg::String();
            message.data = std::to_string(b);
            estimateSpeedPub_->publish(message);
        }
    } else if (result < 0) {
        RCLCPP_ERROR(this->get_logger(), "Error on select: %s", strerror(errno));
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