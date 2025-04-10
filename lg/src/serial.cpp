while (read(serialPort, &rx_char, 1) > 0) {
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

        buffer.clear();  // Clear the buffer for the next message
    } else {
        buffer += rx_char;  // Append character to the buffer
    }
}

ros::spinOnce();  // Allow ROS to process any incoming messages