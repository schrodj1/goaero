#include <ros/ros.h>
#include <std_msgs/UInt16.h>
#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <cstdint>

// Shared variables for PWM values (default to neutral)
uint16_t pulse_us0 = 1500;
uint16_t pulse_us1 = 1500;
uint16_t pulse_us2 = 1500;
uint16_t pulse_us3 = 1500;

// Send a PWM target to a specific servo channel
bool setServoTarget(int fd, uint8_t channel, uint16_t target) {
    uint8_t command[4];
    command[0] = 0x84;
    command[1] = channel;
    command[2] = target & 0x7F;
    command[3] = (target >> 7) & 0x7F;

    ssize_t bytesWritten = write(fd, command, 4);
    if (bytesWritten != 4) {
        std::cerr << "Failed to write command for channel " << static_cast<int>(channel) << "\n";
        return false;
    }
    return true;
}

// ROS subscriber callbacks
void pwmCallback1(const std_msgs::UInt16::ConstPtr& msg) {
    pulse_us0 = msg->data;
}
void pwmCallback2(const std_msgs::UInt16::ConstPtr& msg) {
    pulse_us1 = msg->data;
}
void pwmCallback3(const std_msgs::UInt16::ConstPtr& msg) {
    pulse_us2 = msg->data;
}
void pwmCallback4(const std_msgs::UInt16::ConstPtr& msg) {
    pulse_us3 = msg->data;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "leg_servo_controller");
    ros::NodeHandle nh;

    // Subscribe to PWM commands for each leg
    ros::Subscriber sub1 = nh.subscribe("leg1/pwm", 10, pwmCallback1);
    ros::Subscriber sub2 = nh.subscribe("leg2/pwm", 10, pwmCallback2);
    ros::Subscriber sub3 = nh.subscribe("leg3/pwm", 10, pwmCallback3);
    ros::Subscriber sub4 = nh.subscribe("leg4/pwm", 10, pwmCallback4);

    // Serial port setup
    const char* portName = "/dev/ttyACM0";
    int fd = open(portName, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        std::cerr << "Error opening " << portName << "\n";
        return 1;
    }

    struct termios options;
    if (tcgetattr(fd, &options) < 0) {
        std::cerr << "Error getting port attributes\n";
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
        std::cerr << "Error setting port attributes\n";
        close(fd);
        return 1;
    }

    ros::Rate loop_rate(100);  // 100 Hz

    while (ros::ok()) {
        ros::spinOnce();

        // Convert microseconds to target units (quarter microseconds)
        setServoTarget(fd, 0, pulse_us0 * 4);
        setServoTarget(fd, 1, pulse_us1 * 4);
        setServoTarget(fd, 2, pulse_us2 * 4);
        setServoTarget(fd, 3, pulse_us3 * 4);

        loop_rate.sleep();
    }

    close(fd);
    return 0;
}