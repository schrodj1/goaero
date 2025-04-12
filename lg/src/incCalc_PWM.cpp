#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/UInt16.h>
#include <vector>
#include <cmath>
#include <algorithm>
#include <iomanip>

constexpr double LINK_LENGTH_METERS = 3.0 * 0.0254;  // 3 inches in meters

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

    // Get three points for the plane (we’ll use legs 0, 1, 2)
    auto& p1 = legs[0];
    auto& p2 = legs[1];
    auto& p3 = legs[2];

    // Compute two vectors on the plane
    double v1[3] = {p2.x - p1.x, p2.y - p1.y, p2.z - p1.z};
    double v2[3] = {p3.x - p1.x, p3.y - p1.y, p3.z - p1.z};

    // Compute normal vector (cross product)
    double normal[3] = {
        v1[1]*v2[2] - v1[2]*v2[1],
        v1[2]*v2[0] - v1[0]*v2[2],
        v1[0]*v2[1] - v1[1]*v2[0]
    };

    // Plane: ax + by + cz + d = 0
    double a = normal[0];
    double b = normal[1];
    double c = normal[2];
    double d = -(a * p1.x + b * p1.y + c * p1.z);

    // Reference z value at leg1's position on the plane
    double z_ref = -(a * p1.x + b * p1.y + d) / c;

    for (size_t i = 0; i < legs.size(); ++i) {
        auto& leg = legs[i];

        // Terrain height predicted at this leg position
        double z_plane = -(a * leg.x + b * leg.y + d) / c;

        // How much leg needs to move to match the slope
        double delta_z = z_plane - z_ref;

        // Convert delta_z to leg angle using link length
        double angle_rad = atan2(delta_z / LINK_LENGTH_METERS, 1.0);
        double angle_deg = angle_rad * 180.0 / M_PI;

        // Clamp to safe servo angle range
        const double angleMin = -30.0;
        const double angleMax = 30.0;
        angle_deg = std::clamp(angle_deg, angleMin, angleMax);

        // Set per-leg PWM range
        int pwmMin, pwmMax;
        if (i == 1 || i == 2) {  // Legs 2 & 3 = reversed
            pwmMin = 2080;
            pwmMax = 896;
        } else {                // Legs 1 & 4 = normal
            pwmMin = 896;
            pwmMax = 2080;
        }

        // Map angle to PWM for this leg
        int pwm = static_cast<int>(
            pwmMin + ((angle_deg - angleMin) / (angleMax - angleMin)) * (pwmMax - pwmMin)
        );

        std_msgs::UInt16 pwm_msg;
        pwm_msg.data = pwm;
        leg.pub.publish(pwm_msg);

        // Debug output
        ROS_INFO_STREAM(std::fixed << std::setprecision(2)
            << "Leg " << i+1
            << " | delta_z = " << delta_z << " m"
            << " | angle = " << angle_deg << "°"
            << " | pwm = " << pwm);
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "terrain_leg_controller");
    ros::NodeHandle nh;

    // Set up subscribers and publishers
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
