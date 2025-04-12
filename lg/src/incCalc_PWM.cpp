#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/UInt16.h>
#include <vector>
#include <cmath>
#include <algorithm>

constexpr double LINK_LENGTH_INCHES = 3.0;

struct Leg {
    std::string range_topic;
    std::string pwm;
    double x, y;    // leg position in body frame
    double z;       // terrain height (from range)
    bool received = false;
    ros::Subscriber sub;
    ros::Publisher pub;
};

std::vector<Leg> legs = {
    {"leg1/range", "leg1/command",  0.3,  0.3},
    {"leg2/range", "leg2/command",  0.3, -0.3},
    {"leg3/range", "leg3/command", -0.3,  0.3},
    {"leg4/range", "leg4/command", -0.3, -0.3}
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

    // Target Z offset at each leg position (on the plane)
    double z_ref = -(a * p1.x + b * p1.y + d) / c;  // reference z at leg1

    for (size_t i = 0; i < legs.size(); ++i) {
        auto& leg = legs[i];
        double z_plane = -(a * leg.x + b * leg.y + d) / c;
        double delta_z = z_plane - z_ref;  // relative vertical offset

        // Compute angle from delta_z (Y travel), X = 0 assumed
        double angle_rad = atan2(delta_z / LINK_LENGTH_INCHES, 1.0);
        double angle_deg = angle_rad * 180.0 / M_PI;

        // Clamp angle to safe range
        const double angleMin = -30.0;
        const double angleMax = 30.0;
        if (angle_deg < angleMin) angle_deg = angleMin;
        if (angle_deg > angleMax) angle_deg = angleMax;

        // Convert angle to PWM (900–2000 µs)
        const int pwmMin = 900;
        const int pwmMax = 2000;

        int pwm = static_cast<int>(
        pwmMin + ((angle_deg - angleMin) / (angleMax - angleMin)) * (pwmMax - pwmMin)
        );

        std_msgs::UInt16 pwm_msg;
        pwm_msg.data = pwm;
        leg.pub.publish(pwm_msg);

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
        legs[i].pub = nh.advertise<std_msgs::UInt16>(legs[i].pwm, 1);
    }

    ros::Rate rate(10);
    while (ros::ok()) {
        ros::spinOnce();
        calculateLegCommands();
        rate.sleep();
    }

    return 0;
}
