#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/UInt16.h>
#include <vector>
#include <cmath>
#include <algorithm>
#include <iomanip>

constexpr double LINK_LENGTH_METERS = 3.0 * 0.0254;  // 3 inches in meters
constexpr double MAX_ANGLE_DEG = 30.0;
constexpr double MIN_ANGLE_DEG = -30.0;

struct leg {
    std::string range_topic;
    std::string pwm_topic;
    double x, y;    // leg position in body frame
    double z;       // terrain height (from range)
    bool received = false;
    ros::Subscriber sub;
    ros::Publisher pub;
};

std::vector<leg> legs = {
    {"leg1/range", "leg1/pwm_msg",  0.3,  0.3},
    {"leg2/range", "leg2/pwm_msg",  0.3, -0.3},
    {"leg3/range", "leg3/pwm_msg", -0.3,  0.3},
    {"leg4/range", "leg4/pwm_msg", -0.3, -0.3}
};

void rangeCallback(const sensor_msgs::Range::ConstPtr& msg, int index) {
    legs[index].z = -msg->range;  // terrain height: negative range
    legs[index].received = true;
}

void calculatelegCommands() {
    // Make sure recieved values for all legs
    if (!std::all_of(legs.begin(), legs.end(), [](const leg& l){ return l.received; }))
        return;

    // Use the leg with the min range (closest to ground) as reference
    size_t ref_index = 0;
    double min_range = legs[0].z;
    for (size_t i = 1; i < legs.size(); ++i) {
        double current_range = -legs[i].z;
        if (current_range < min_range) {
            min_range = current_range;
            ref_index = i;
        }
    }

    // find servo angles using closest range as 30 degrees
    int angles[4];
    int height_diff;
    for (size_t i = 0; i < legs.size(); ++i){
        height_diff = -legs[i].z - min_range;
        if (i == ref_index){
            angles[i] = MIN_ANGLE_DEG;
        } else {
            angles[i] = MIN_ANGLE_DEG- ((180/M_PI)*(acos(height_diff/LINK_LENGTH_METERS)));
        }
        if(angles[i] > MAX_ANGLE_DEG){
            angles[i] = MAX_ANGLE_DEG;
        }
    }


    // Calculate and publish leg commands
    for (size_t i = 0; i < legs.size(); ++i) {

        int angles[4];
        int height_diff[4];
        height_diff[i] = -legs[i].z - min_range;
        if (i == ref_index){
            angles[i] = MIN_ANGLE_DEG;
        } else {
            angles[i] = MIN_ANGLE_DEG- ((180/M_PI)*(acos(height_diff[i]/LINK_LENGTH_METERS)));
        }
        if(angles[i] > MAX_ANGLE_DEG){
            angles[i] = MAX_ANGLE_DEG;
        }

        // Set per-leg PWM range
        int pwmMin, pwmMax;
        if (i == 1 || i == 2) {  // legs 2 & 3 = reversed
            pwmMin = 1880;
            pwmMax = 1096;
        } else {                // legs 1 & 4 = normal
            pwmMin = 1096;
            pwmMax = 1880;
        }

        // Map angle to PWM
        int pwm = static_cast<int>(
            pwmMin + ((angles[i] - MIN_ANGLE_DEG) / (MAX_ANGLE_DEG - MIN_ANGLE_DEG)) * (pwmMax - pwmMin)
        );

        std_msgs::UInt16 pwm_msg;
        pwm_msg.data = pwm;
        leg.pub.publish(pwm_msg);

        ROS_INFO_STREAM(std::fixed << std::setprecision(2)
            << "leg " << i+1
            << (i == ref_index ? " [REFERENCE]" : "")
            << " | range = " << -leg.z << " m"
            << " | delta_z = " << height_diff[i] << " m"
            << " | angle = " << angles[i] << "°"
            << " | pwm = " << pwm);
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "terrain_leg_controller");
    ros::NodeHandle nh;

    for (size_t i = 0; i < legs.size(); ++i) {
        legs[i].sub = nh.subscribe<sensor_msgs::Range>(
            legs[i].range_topic, 1,
            boost::bind(&rangeCallback, _1, i)
        );
        legs[i].pub = nh.advertise<std_msgs::UInt16>(legs[i].pwm_topic, 1);
    }

    ros::Rate rate(10);
    while (ros::ok()) {
        ros::spinOnce();
        calculatelegCommands();
        rate.sleep();
    }

    return 0;
}
