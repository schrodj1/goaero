#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Byte.h>
#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <cstdint>
#include <filesystem>
#include <string>

namespace fs = std::filesystem;

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

int fd;

bool setServoTarget(int fd, uint8_t channel, uint16_t target) {
    uint8_t command[4] = {0x84, channel, target & 0x7F, (target >> 7) & 0x7F};
    return (write(fd, command, 4) == 4);
}



long map(long x, long in_min, long in_max, long out_min, long out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void angleCallback(const std_msgs::Float32::ConstPtr& msg, int channel, ros::Publisher& pub) {
    int pwm = map(static_cast<int>(msg->data), 0, 180, 1000, 2000) * 4;
    std_msgs::Byte pwm_msg;
    pwm_msg.data = pwm / 4;
    pub.publish(pwm_msg);
    if (!setServoTarget(fd, channel, pwm)) {
        ROS_ERROR("Failed to send PWM to servo on channel %d", channel);
    } else {
        ROS_INFO("Set channel %d to PWM %d", channel, pwm / 4);
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "servo_controller");
    ros::NodeHandle nh;

    std::string portName;
    try {
        portName = findMaestroCommandPort();
        ROS_INFO("Detected Maestro Command Port: %s", portName.c_str());
    } catch (const std::exception& e) {
        ROS_ERROR("%s", e.what());
        return 1;
    }

    fd = open(portName.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        ROS_ERROR("Error opening %s", portName.c_str());
        return 1;
    }

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

    ros::Publisher pub_leg1 = nh.advertise<std_msgs::Byte>("leg1/comAngle", 10);
    ros::Publisher pub_leg2 = nh.advertise<std_msgs::Byte>("leg2/comAngle", 10);
    ros::Publisher pub_leg3 = nh.advertise<std_msgs::Byte>("leg3/comAngle", 10);
    ros::Publisher pub_leg4 = nh.advertise<std_msgs::Byte>("leg4/comAngle", 10);

    ros::Subscriber sub_leg1 = nh.subscribe<std_msgs::Float32>("leg1/numAngle", 1000,
        boost::bind(&angleCallback, _1, 0, boost::ref(pub_leg1)));
    ros::Subscriber sub_leg2 = nh.subscribe<std_msgs::Float32>("leg2/numAngle", 1000,
        boost::bind(&angleCallback, _1, 1, boost::ref(pub_leg2)));
    ros::Subscriber sub_leg3 = nh.subscribe<std_msgs::Float32>("leg3/numAngle", 1000,
        boost::bind(&angleCallback, _1, 2, boost::ref(pub_leg3)));
    ros::Subscriber sub_leg4 = nh.subscribe<std_msgs::Float32>("leg4/numAngle", 1000,
        boost::bind(&angleCallback, _1, 3, boost::ref(pub_leg4)));

    ros::spin();
    close(fd);
    return 0;
}

