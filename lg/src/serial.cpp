#include <ros/ros.h>
#include <std_msgs/Int32MultiArray.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <cstring>
#include <sstream>
#include <vector>

int initSerial(int serialPort) {
    struct termios options;

    if (tcgetattr(serialPort, &options) != 0) {
        return 1;
    }

    // Set baud rate
    if (cfsetispeed(&options, B9600) != 0 || cfsetospeed(&options, B9600) != 0) {
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
    
    

    // Open the serial port
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

    char rx_buf[256];  // Buffer for incoming data

    ros::Rate loop_rate(10);  // Loop rate at 10 Hz

    while (ros::ok()) {
        memset(rx_buf, 0, sizeof(rx_buf));  // Clear the buffer

        int bytesRead = read(serialPort, rx_buf, sizeof(rx_buf) - 1);  // Read data from the serial port
        if (bytesRead > 0) {
            rx_buf[bytesRead] = '\0';  // Null-terminate the string
            std::cout << "Received: " << rx_buf << std::endl;

            // Parse the received data into integers
            std::vector<int> sensorValues;
            std::stringstream ss(rx_buf);
            int value;

            while (ss >> value) {
                sensorValues.push_back(value);
                if (ss.peek() == ' ') ss.ignore();  // Ignore spaces
            }

            // Only proceed if we have exactly 8 values
            if (sensorValues.size() == 8) {
                // Create a message to publish
                std_msgs::Int32MultiArray msg;
                msg.data = sensorValues;

                // Publish the sensor values to the ROS topic
                sensor_pub.publish(msg);
            }
        }

        ros::spinOnce();  // Allow ROS to process any incoming messages
        loop_rate.sleep();  // Sleep to maintain loop rate
    }

    close(serialPort);  // Close the serial port when done
    return 0;
}
