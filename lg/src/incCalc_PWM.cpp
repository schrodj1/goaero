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

struct Leg {
    std::string range_topic;
    std::string pwm_topic;
    double x, y;    // leg position in body frame
    double z;       // terrain height (from range)
    bool received = false;
    ros::Subscriber sub;
    ros::Publisher pub;
};

std::vector<Leg> legs = {
    {"leg1/range", "leg1/pwm_msg",  0.3,  0.3},
    {"leg2/range", "leg2/pwm_msg",  0.3, -0.3},
    {"leg3/range", "leg3/pwm_msg", -0.3,  0.3},
    {"leg4/range", "leg4/pwm_msg", -0.3, -0.3}
};

void rangeCallback(const sensor_msgs::Range::ConstPtr& msg, int index) {
    legs[index].z = -msg->range;  // terrain is below the body
    legs[index].received = true;
}

void calculateLegCommands() {
    if (!std::all_of(legs.begin(), legs.end(), [](const Leg& l){ return l.received; }))
        return;

    // 1. Fit a plane to the terrain
    auto& p1 = legs[0];
    auto& p2 = legs[1];
    auto& p3 = legs[2];

    double v1[3] = {p2.x - p1.x, p2.y - p1.y, p2.z - p1.z};
    double v2[3] = {p3.x - p1.x, p3.y - p1.y, p3.z - p1.z};

    double normal[3] = {
        v1[1]*v2[2] - v1[2]*v2[1],
        v1[2]*v2[0] - v1[0]*v2[2],
        v1[0]*v2[1] - v1[1]*v2[0]
    };

    double a = normal[0];
    double b = normal[1];
    double c = normal[2];
    double d = -(a * p1.x + b * p1.y + c * p1.z);

    // 2. Find leg with smallest range (i.e., highest z = most ground contact)
    size_t ref_index = 0;
    double min_range = -legs[0].z;  // z is negative
    for (size_t i = 1; i < legs.size(); ++i) {
        if (-legs[i].z < min_range) {
            min_range = -legs[i].z;
            ref_index = i;
        }
    }
    const auto& ref_leg = legs[ref_index];
    double z_ref = -(a * ref_leg.x + b * ref_leg.y + d) / c;

    // 3. Publish PWM for each leg
    for (size_t i = 0; i < legs.size(); ++i) {
        auto& leg = legs[i];

        double z_plane = -(a * leg.x + b * leg.y + d) / c;
        double delta_z = z_plane - z_ref;

        double angle_deg;
        if (i == ref_index) {
            angle_deg = MAX_ANGLE_DEG;  // Full extension
        } else {
            // delta_z is the difference in terrain height relative to the "touchdown leg"
            double angle_rad = atan2(delta_z / LINK_LENGTH_METERS, 1.0);
            angle_deg = MAX_ANGLE_DEG - (angle_rad * 180.0 / M_PI);
        }

        angle_deg = std::clamp(angle_deg, MIN_ANGLE_DEG, MAX_ANGLE_DEG);

        // Set per-leg PWM range
        int pwmMin, pwmMax;
        if (i == 1 || i == 2) {  // Legs 2 & 3 = reversed
            pwmMin = 2080;
            pwmMax = 896;
        } else {                // Legs 1 & 4 = normal
            pwmMin = 896;
            pwmMax = 2080;
        }

        // Map angle to PWM
        int pwm = static_cast<int>(
            pwmMin + ((angle_deg - MIN_ANGLE_DEG) / (MAX_ANGLE_DEG - MIN_ANGLE_DEG)) * (pwmMax - pwmMin)
        );

        std_msgs::UInt16 pwm_msg;
        pwm_msg.data = pwm;
        leg.pub.publish(pwm_msg);

        ROS_INFO_STREAM(std::fixed << std::setprecision(2)
            << "Leg " << i+1
            << (i == ref_index ? " [REFERENCE]" : "")
            << " | delta_z = " << delta_z << " m"
            << " | angle = " << angle_deg << "°"
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
        calculateLegCommands();
        rate.sleep();
    }

    return 0;
}
