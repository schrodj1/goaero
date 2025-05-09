#include <ros/ros.h>
#include <std_msgs/Int32MultiArray.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <cstring>
#include <sstream>
#include <vector>
#include <string>

// Function to initialize the serial port
int initSerial(int serialPort) {
    struct termios options;

    if (tcgetattr(serialPort, &options) != 0) {
        return 1;
    }

    // Set baud rate
    if (cfsetispeed(&options, B115200) != 0 || cfsetospeed(&options, B115200) != 0) {
        return 2;
    }

    // 8n1 (no parity, 1 stop bit, 8 data bits)
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_cflag &= ~CRTSCTS;
    options.c_cflag |= CREAD | CLOCAL;

    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_iflag &= ~(IXON | IXOFF | IXANY);
    options.c_oflag &= ~OPOST;

    // Apply settings
    if (tcsetattr(serialPort, TCSANOW, &options) != 0) {
        return 3;
    }

    return 0;
} 

int main(int argc, char** argv) {
    // Initialize the ROS node
    ros::init(argc, argv, "serial_to_ros_node");
    ros::NodeHandle nh;

    // Create a publisher to publish sensor data
    ros::Publisher sensor_pub = nh.advertise<std_msgs::Int32MultiArray>("sensor_values", 10);

    // Open the serial port for the Arduino
    const char *portName = "/dev/ttyACM0";
    int serialPort = open(portName, O_RDWR | O_NOCTTY | O_NDELAY);

    if (serialPort == -1) {
        ROS_ERROR("Unable to open serial port.");
        return 0;
    }

    if (initSerial(serialPort) != 0) {
        close(serialPort);
        ROS_ERROR("Failed to initialize serial port.");
        return 0;
    }

    std::cout << "Serial port opened. Waiting for data..." << std::endl;

    std::string buffer;  // Buffer to store incoming data
    char rx_char;        // Character read from the serial port

    ros::Rate loop_rate(100);

    while (ros::ok()) {
        // Read one character at a time
        while (read(serialPort, &rx_char, 1) > 0) {
            // Check for newline to indicate the end of a message
            if (rx_char == '\n') {
                // Process the complete message
                std::stringstream ss(buffer);
                std::vector<int> sensorValues;
                int value;

                while (ss >> value) {
                    sensorValues.push_back(value);
                }

                // Create a message to publish
                std_msgs::Int32MultiArray msg;
                msg.data = sensorValues;

                // Publish the sensor values to the ROS topic
                sensor_pub.publish(msg);
                // ROS_INFO("Published sensor values: [%s]", buffer.c_str());

                buffer.clear();  // Clear the buffer for the next message
            } else {
                buffer += rx_char;  // Append character to the buffer
            }
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    close(serialPort);  // Close the serial port when done
    return 0;
}