#include <ros/ros.h>
#include <std_msgs/Int32MultiArray.h>
#include <sensor_msgs/Range.h>

ros::Publisher range_pub_leg1;
ros::Publisher range_pub_leg2;
ros::Publisher range_pub_leg3;
ros::Publisher range_pub_leg4;

// Callback function to handle incoming sensor data
void sensorCallback(const std_msgs::Int32MultiArray::ConstPtr& msg) {
    // Ensure we have at least 4 values in the array
    if (msg->data.size() >= 4) {
        // Iterate over the first 4 sensor values and publish to the corresponding leg topic
        for (int i = 0; i < 4; ++i) {
            // Extract each value (assumed to be in millimeters)
            int sensor_value = msg->data[i];

            // Convert the received value to meters (assuming the sensor value is in millimeters)
            float range = static_cast<float>(sensor_value) / 100.0f;  // Convert to meters

            // Create a Range message
            sensor_msgs::Range range_msg;
            range_msg.header.stamp = ros::Time::now();
            range_msg.header.frame_id = "sensor_frame";  // Set the reference frame name (change as needed)
            range_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;  // Assuming ultrasonic sensor type
            range_msg.field_of_view = 0.1;  // Set the field of view (adjust based on sensor type)
            range_msg.min_range = 0.02;     // Set the minimum measurable range (e.g., 2 cm)
            range_msg.max_range = 4.0;      // Set the maximum measurable range (e.g., 4 meters)
            range_msg.range = range;        // Set the range value in meters

            // Publish the Range message to the correct topic based on the index
            switch (i) {
                case 0:
                    range_pub_leg1.publish(range_msg);
                    ROS_INFO("Published range for Leg 1: %.2f meters", range);
                    break;
                case 1:
                    range_pub_leg2.publish(range_msg);
                    ROS_INFO("Published range for Leg 2: %.2f meters", range);
                    break;
                case 2:
                    range_pub_leg3.publish(range_msg);
                    ROS_INFO("Published range for Leg 3: %.2f meters", range);
                    break;
                case 3:
                    range_pub_leg4.publish(range_msg);
                    ROS_INFO("Published range for Leg 4: %.2f meters", range);
                    break;
            }
        }
    } else {
        ROS_WARN("Received sensor data with less than 4 values. Ignoring this message.");
    }
}

int main(int argc, char** argv) {
    // Initialize the ROS node
    ros::init(argc, argv, "rangefinder_node");
    ros::NodeHandle nh;

    // Create separate publishers for each leg's range data
    range_pub_leg1 = nh.advertise<sensor_msgs::Range>("leg1/range", 10);
    range_pub_leg2 = nh.advertise<sensor_msgs::Range>("leg2/range", 10);
    range_pub_leg3 = nh.advertise<sensor_msgs::Range>("leg3/range", 10);
    range_pub_leg4 = nh.advertise<sensor_msgs::Range>("leg4/range", 10);

    // Subscribe to the sensor_values topic published by the serial node
    ros::Subscriber sensor_sub = nh.subscribe("sensor_values", 10, sensorCallback);

    // Enter the ROS event loop to handle incoming messages and process callbacks
    ros::spin();

    return 0;
}
