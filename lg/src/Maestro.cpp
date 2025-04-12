#include <ros/ros.h>
#include <std_msgs/UInt16.h>
#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <cstdint>
#include <filesystem>
#include <string>

namespace fs = std::filesystem;

// Function to find the Maestro command port dynamically
std::string findMaestroCommandPort() {
    const std::string path = "/dev/serial/by-id/";
    for (const auto& entry : fs::directory_iterator(path)) {
        std::string name = entry.path().filename();
        if (name.find("Pololu_Micro_Maestro") != std::string::npos &&
            name.find("if00") != std::string::npos) {
            return fs::canonical(entry.path()).string();
        }
    }
    throw std::runtime_error("Maestro Command Port not found!");
}

int fd; // File descriptor for the Maestro serial connection

// Function to send a PWM signal to the Maestro controller
bool setServoTarget(int fd, uint8_t channel, uint16_t target) {
    uint8_t command[4] = {0x84, channel, target & 0x7F, (target >> 7) & 0x7F};
    return (write(fd, command, 4) == 4);
}

// Callback function to handle incoming PWM signals
void pwmCallback(const std_msgs::UInt16::ConstPtr& msg, int channel) {
    uint16_t pwm = msg->data*4; // PWM value received from the topic
    if (!setServoTarget(fd, channel, pwm)) {
        ROS_ERROR("Failed to send PWM to servo on channel %d", channel);
    } else {
        ROS_INFO("Set channel %d to PWM %d", channel, pwm);
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "servo_controller");
    ros::NodeHandle nh;

    // Find the Maestro command port
    std::string portName;
    try {
        portName = findMaestroCommandPort();
        ROS_INFO("Detected Maestro Command Port: %s", portName.c_str());
    } catch (const std::exception& e) {
        ROS_ERROR("%s", e.what());
        return 1;
    }

    // Open the serial port
    fd = open(portName.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        ROS_ERROR("Error opening %s", portName.c_str());
        return 1;
    }

    // Configure the serial port
    struct termios options;
    if (tcgetattr(fd, &options) < 0) {
        ROS_ERROR("Error getting port attributes");
        close(fd);
        return 1;
    }

    cfsetispeed(&options, B9600);
    cfsetospeed(&options, B9600);
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_cflag |= CREAD | CLOCAL;
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_oflag &= ~OPOST;
    tcflush(fd, TCIOFLUSH);
    if (tcsetattr(fd, TCSANOW, &options) < 0) {
        ROS_ERROR("Error setting port attributes");
        close(fd);
        return 1;
    }

    // Set up subscribers for PWM signals
    ros::Subscriber sub_leg1 = nh.subscribe<std_msgs::UInt16>("leg1/pwm_msg", 1000,
        boost::bind(&pwmCallback, _1, 0));
    ros::Subscriber sub_leg2 = nh.subscribe<std_msgs::UInt16>("leg2/pwm_msg", 1000,
        boost::bind(&pwmCallback, _1, 1));
    ros::Subscriber sub_leg3 = nh.subscribe<std_msgs::UInt16>("leg3/pwm_msg", 1000,
        boost::bind(&pwmCallback, _1, 2));
    ros::Subscriber sub_leg4 = nh.subscribe<std_msgs::UInt16>("leg4/pwm_msg", 1000,
        boost::bind(&pwmCallback, _1, 3));

    // Enter the ROS event loop
    ros::spin();

    // Close the serial port when the node shuts down
    close(fd);
    return 0;
}