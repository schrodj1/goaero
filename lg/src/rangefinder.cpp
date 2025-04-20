#include <ros/ros.h>
#include <std_msgs/Int32MultiArray.h>
#include <sensor_msgs/Range.h>
#include <geometry_msgs/Vector3.h>


ros::Publisher range_pub_leg1;
ros::Publisher range_pub_leg2;
ros::Publisher range_pub_leg3;
ros::Publisher range_pub_leg4;

ros:: Publisher angle_pub_roll;
ros:: Publisher angle_pub_pitch;
ros:: Publisher angle_pub_yaw;

// Callback function to handle incoming sensor data
void sensorCallback(const std_msgs::Int32MultiArray::ConstPtr& msg) {
    // Ensure we have at least 10 values in the array
    if (msg->data.size() >= 10) {
        // Iterate over the first 4 sensor values and publish to the corresponding leg topic
        for (int i = 0; i < 4; ++i) {
            // Extract each value (assumed to be in millimeters)
            int sensor_value = msg->data[i];

            // convert value to float
            float range = static_cast<float>(sensor_value); 

            // Create a Range message
            sensor_msgs::Range range_msg;
            range_msg.header.stamp = ros::Time::now();
            range_msg.header.frame_id = "tof_sensor_frame";  // Set the reference frame name (change as needed)
            range_msg.radiation_type = sensor_msgs::Range::INFRARED;  // ToF sensors use infrared radiation
            range_msg.field_of_view = 0.05;  // Set the field of view (adjust based on VL53L4CB specs)
            range_msg.min_range = 1;      // Set the minimum measurable range (e.g., 1 mm)
            range_msg.max_range = 300;       // Set the maximum measurable range (e.g., 3 meters)
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

        for (int i = 7; i < 10; ++i) {
            // Extract each value (assumed to be in radians)
            int sensor_value = msg->data[i];

            // convert value to float and save in array
            float angle[3];
            angle[i] = static_cast<float>(sensor_value); 
        }

        // Create a Vector3 message
        geometry_msgs::Vector3 angle_msg;
        angle_msg.header.stamp = ros::Time::now();
        angle_msg.header.frame_id = "IMU_sensor_frame";  // Set the reference frame name (change as needed)
        angle_msg.x = angle[1];  // Set the x component
        angle_msg.y = angle[2];  // Set the y component
        angle_msg.z = angle[3];  // Set the z component

        // publish angle topics
        angle_pub_roll.publish(angle_msg.x);  // Publish the roll angle
        angle_pub_pitch.publish(angle_msg.y);  // Publish the pitch angle
        angle_pub_yaw.publish(angle_msg.z);  // Publish the yaw angle
            
    } else {
        // ROS_WARN("Received sensor data with less than 10 values. Ignoring this message.");
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

    angle_pub_roll = nh.advertise<geometry_msgs::Vector3>("roll_angle", 10);
    angle_pub_pitch = nh.advertise<geometry_msgs::Vector3>("pitch_angle", 10);
    angle_pub_yaw = nh.advertise<geometry_msgs::Vector3>("yaw_angle", 10);
    
    // Subscribe to the sensor_values topic published by the serial node
    ros::Subscriber sensor_sub = nh.subscribe("sensor_values", 10, sensorCallback);

    // Enter the ROS event loop to handle incoming messages and process callbacks
    ros::spin();

    return 0;
}