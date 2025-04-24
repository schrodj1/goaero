#include <ros/ros.h>
#include <std_msgs/Int32MultiArray.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Int32.h>  // Include for publishing single integer values

ros::Publisher range_pub_leg1;
ros::Publisher range_pub_leg2;
ros::Publisher range_pub_leg3;
ros::Publisher range_pub_leg4;
ros::Publisher pitch_pub;  // New publisher for the value at index 7
ros::Publisher roll_pub;  // New publisher for the value at index 8

// Callback function to handle incoming sensor data
void sensorCallback(const std_msgs::Int32MultiArray::ConstPtr& msg) {
    // Ensure we have at least 4 values in the array
    if (msg->data.size() >= 4) {
        // Iterate over the first 4 sensor values and publish to the corresponding leg topic
        for (int i = 0; i < 4; ++i) {
            // Extract each value (assumed to be in millimeters)
            int sensor_value = msg->data[i];

            // (assuming the sensor value is in millimeters)
            float range = static_cast<float>(sensor_value); 

            // Create a Range message
            sensor_msgs::Range range_msg;
            range_msg.header.stamp = ros::Time::now();
            range_msg.header.frame_id = "tof_sensor_frame";  // Set the reference frame name (change as needed)
            range_msg.radiation_type = sensor_msgs::Range::INFRARED;  // ToF sensors use infrared radiation
            range_msg.field_of_view = 0.05;  // Set the field of view (adjust based on VL53L4CB specs)
            range_msg.min_range = 0.01;      // Set the minimum measurable range (e.g., 1 cm)
            range_msg.max_range = 4.0;       // Set the maximum measurable range (e.g., 4 meters)
            range_msg.range = range;         // Set the range value in meters

            // Publish the Range message to the correct topic based on the index
            switch (i) {
                case 0:
                    range_pub_leg1.publish(range_msg);
                    // ROS_INFO("Published range for Leg 1: %.2f meters", range);
                    break;
                case 1:
                    range_pub_leg2.publish(range_msg);
                    // ROS_INFO("Published range for Leg 2: %.2f meters", range);
                    break;
                case 2:
                    range_pub_leg3.publish(range_msg);
                    // ROS_INFO("Published range for Leg 3: %.2f meters", range);
                    break;
                case 3:
                    range_pub_leg4.publish(range_msg);
                    // ROS_INFO("Published range for Leg 4: %.2f meters", range);
                    break;
            }
        }
    }

    // Check if the array has at least 9 values to access index 8
    if (msg->data.size() > 9) {
        int pitch_value = msg->data[7];  // Extract the value at index 7

        // Publish the value at index 7
        std_msgs::Int32 pitch_value_msg;
        pitch_value_msg.data = pitch_value;
        pitch_pub.publish(pitch_value_msg);
        ROS_INFO("IMU pitch value: %d", pitch_value);
 
        int roll_value = msg->data[8];  // Extract the value at index 8

        // Publish the value at index 8
        std_msgs::Int32 roll_value_msg;
        roll_value_msg.data = roll_value;
        roll_pub.publish(roll_value_msg);
        //ROS_INFO("IMU roll value: %d", roll_value);


        // ROS_INFO("Published extra value at index 8: %d", extra_value);
    } else {
        // ROS_WARN("Received sensor data with less than 9 values. Cannot publish extra value.");
    }
}

int main(int argc, char** argv) {
    // Initialize the ROS node
    ros::init(argc, argv, "rangefinder_node");
    ros::NodeHandle nh;

    // Create separate publishers for each leg's range data
    range_pub_leg1 = nh.advertise<sensor_msgs::Range>("leg1/range", 5);
    range_pub_leg2 = nh.advertise<sensor_msgs::Range>("leg2/range", 5);
    range_pub_leg3 = nh.advertise<sensor_msgs::Range>("leg3/range", 5);
    range_pub_leg4 = nh.advertise<sensor_msgs::Range>("leg4/range", 5);

    // Create a publisher for the extra value at index 7
    pitch_pub = nh.advertise<std_msgs::Int32>("pitch_value", 5);
    // Create a publisher for the extra value at index 8
    roll_pub = nh.advertise<std_msgs::Int32>("roll_value", 5);

    // Subscribe to the sensor_values topic published by the serial node
    ros::Subscriber sensor_sub = nh.subscribe("sensor_values", 10, sensorCallback);

    // Enter the ROS event loop to handle incoming messages and process callbacks
    ros::spin();

    return 0;
}