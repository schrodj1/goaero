
#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Int32.h>
#include <vector>
#include <cmath>
#include <algorithm>
#include <iomanip>


template <typename T>
T clamp(T value, T min, T max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

// set values for horizontal link length, and retracted and extended angles
constexpr double LINK_LENGTH_METERS = 3.0 * 25.4;  // 3 inches in meters
constexpr double RETRACTED_ANGLE = 40.0;
constexpr double EXTENDED_ANGLE = -40.0;

// set up servo angle vector
double angles[4] = {RETRACTED_ANGLE, RETRACTED_ANGLE, RETRACTED_ANGLE, RETRACTED_ANGLE};

double rollIMU = 0.0;
double pitchIMU = 0.0;
double w = 165.113 / 2; // distance between legs left to right
double l = LINK_LENGTH_METERS; // length of horizontal link
double D = 213.7*2; // distance between legs front to back
double RAD2DEG = 180/M_PI; //for my sanity
double DEG2RAD = M_PI/180; //dear God pls work

// create struct to store leg position and range data
struct leg_struct {
    std::string range_topic;
    std::string pwm_topic;
    double x, y;    // leg position in body frame
    double z;       // terrain height (from range)
    bool received = false;
    ros::Subscriber sub;
    ros::Publisher pub;
};

// vector to assign range topic and pwn topic and define x and y positions
std::vector<leg_struct> legs = {
    {"leg1/range", "leg1/pwm_msg",  213.7 , -39.315},
    {"leg2/range", "leg2/pwm_msg", -213.7, -39.315},
    {"leg3/range", "leg3/pwm_msg", 213.7,  39.315},
    {"leg4/range", "leg4/pwm_msg", -213.7, 39.315 }
};

// reads range messages and saves them in correct array instance 
void rangeCallback(const sensor_msgs::Range::ConstPtr& msg, int index) {
    legs[index].z = -msg->range;  // terrain height: negative range
    legs[index].received = true;
};

// callback to update roll value from the subscribed topic
void pitchCallback(const std_msgs::Int32::ConstPtr& msg) {
    pitchIMU = static_cast<double>(msg->data);  // update the roll value
};

// callback to update roll value from the subscribed topic
void rollCallback(const std_msgs::Int32::ConstPtr& msg) {
    rollIMU = static_cast<double>(msg->data);  // update the roll value
};

// calculate leg positions for each leg
void calculatelegCommands() {
    // Make sure recieved values for all legs
    if (!std::all_of(legs.begin(), legs.end(), [](const leg_struct& l){ return l.received; }))
        return;
 

    double hdiffF = legs[2].z-legs[0].z; // finds height diff from sensor 1&3
    double hdiffB = legs[3].z-legs[1].z; // finds height diff from sensor 2&4
    double hdiffFB = ((-legs[2].z+legs[3].z)+(-legs[0].z+legs[1].z))/2; // finds height diff by averaging height differences
    
    double rollF = atan(hdiffF/(39.315*2)) + rollIMU*DEG2RAD;
    double rollB = atan(hdiffB/(39.315*2)) + rollIMU*DEG2RAD;
    double pitch = (atan(hdiffFB/(D)) - pitchIMU*DEG2RAD); // find pitch angle
    
    // Front legs (phi1 and phi3)
    if (rollF == 0) {
        angles[0] = RAD2DEG * asin(D*tan(pitch) / (2.0 * l));
        angles[2] = RAD2DEG * asin(D*tan(pitch) / (2.0 * l));
    } else {
        angles[0] = RAD2DEG * (rollF + asin(((w + (D * tan(pitch)) / (2.0 * sin(rollF))) * sin(rollF)) / l));
        angles[2] = -RAD2DEG * (rollF + asin(((w - (D * tan(pitch)) / (2.0 * sin(rollF))) * sin(rollF)) / l));
    // } else {
    //     double sin(rollF) = std::max(std::abs(sin(rollF)), epsilon);
    //     angles[0] = RAD2DEG * (rollF + asin((((w + (D * tan(pitch)) / (2.0 * sin(rollF))) * sin(rollF)) / l, -1.0, 1.0)));
    //     angles[2] = -RAD2DEG * (rollF + asin((((w - (D * tan(pitch)) / (2.0 * sin(rollF))) * sin(rollF)) / l, -1.0, 1.0)));
    // }
    }
    // Back legs (phi2 and phi4)
    // if (std::abs(rollB) < epsilon) {
    //     double safe_asin_input = clamp((D * tan(pitch)) / (2.0 * l), -1.0, 1.0);
    //     angles[1] = -RAD2DEG * asin(safe_asin_input);
    //     angles[3] = -RAD2DEG * asin(safe_asin_input);
    // } else if (rollB < 0) {
    //     double rollB_sin = std::max(std::abs(sin(rollB)), epsilon);
    //     angles[1] = RAD2DEG * (rollB + asin(clamp(((w - (D * tan(pitch)) / (2.0 * rollB_sin)) * rollB_sin) / l, -1.0, 1.0)));
    //     angles[3] = -RAD2DEG * (rollB + asin(clamp(((w + (D * tan(pitch)) / (2.0 * rollB_sin)) * rollB_sin) / l, -1.0, 1.0)));
    // } else {
    //     double rollB_sin = std::max(std::abs(sin(rollB)), epsilon);
    //     angles[1] = RAD2DEG * (rollB + asin(clamp(((w + (D * tan(pitch)) / (2.0 * rollB_sin)) * rollB_sin) / l, -1.0, 1.0)));
    //     angles[3] = -RAD2DEG * (rollB + asin(clamp(((w - (D * tan(pitch)) / (2.0 * rollB_sin)) * rollB_sin) / l, -1.0, 1.0)));
    // }

    if (rollB == 0) {
        angles[1] = -RAD2DEG * asin(D*tan(pitch) / (2.0 * l));
        angles[3] = -RAD2DEG * asin(D*tan(pitch) / (2.0 * l));
    } else {
        angles[1] = RAD2DEG * (rollB + asin(((w - (D * tan(pitch)) / (2.0 * sin(rollB))) * sin(rollB)) / l));
        angles[3] = -RAD2DEG * (rollB + asin(((w + (D * tan(pitch)) / (2.0 * sin(rollB))) * sin(rollB)) / l));
    }
    
    
    
    
        for (int i =0; i<4; ++i){
    //clamp angles between 45 and -45
        angles[i] = clamp(angles[i], EXTENDED_ANGLE, RETRACTED_ANGLE);

        // Set per-leg PWM range
        int pwmMin, pwmMax;
        if (i == 0) {  // legs 2 & 3 = reversed
            pwmMin = 896;
            pwmMax = 1713.75;
        } else if (i == 1) {                // legs 1 & 4 = normal
            pwmMin = 2070;
            pwmMax = 1250;
        } else if (i == 2) {                // legs 1 & 4 = normal
            pwmMin = 2070;
            pwmMax = 1191;
        } else  {                // legs 1 & 4 = normal
            pwmMin = 1000;
            pwmMax = 1801.5;
        }

        // Map angle to PWM
        int pwm = static_cast<int>(
            pwmMin + ((angles[i] - EXTENDED_ANGLE) / (RETRACTED_ANGLE - EXTENDED_ANGLE)) * (pwmMax - pwmMin));

        // create msg and publish
        std_msgs::UInt16 pwm_msg;
        pwm_msg.data = pwm;
        legs[i].pub.publish(pwm_msg);

        // // print data to console for troubleshooting
        // ROS_INFO_STREAM(std::fixed << std::setprecision(2)
        //    << " | rollF = " << rollF*RAD2DEG << "°"
        //    << " | rollB = " << rollB*RAD2DEG << "°"
        //    << " | Pitch = " << pitch*RAD2DEG << "°"
        //    << " | Angle 1 = " << angles[0] << "°"
        //    << " | Angle 2 = " << angles[1] << "°"
        //    << " | Angle 3 = " << angles[2] << "°"
        //    << " | Angle 4 = " << angles[3] << "°"
        //    );
    }

}

int main(int argc, char** argv) {
    ros::init(argc, argv, "terrain_leg_controller");
    ros::NodeHandle nh;

    // Subscribe to roll_value topic
    ros::Subscriber roll_sub = nh.subscribe("roll_value", 1, rollCallback);

    // Subscribe to roll_value topic
    ros::Subscriber pitch_sub = nh.subscribe("pitch_value", 1, pitchCallback);

    for (size_t i = 0; i < legs.size(); ++i) {
        legs[i].sub = nh.subscribe<sensor_msgs::Range>(
            legs[i].range_topic, 1,
            boost::bind(&rangeCallback, _1, i)
        );
        legs[i].pub = nh.advertise<std_msgs::UInt16>(legs[i].pwm_topic, 1);
    }

    ros::Rate rate(100);
    while (ros::ok()) {
        ros::spinOnce();
        calculatelegCommands();
        rate.sleep();
    }

    return 0;
}
