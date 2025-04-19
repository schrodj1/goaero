// build plane and use vectors to match transform
// from foot position to TOF Sensor Data
#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/UInt16.h>
#include <vector>
#include <cmath>
#include <algorithm>
#include <iomanip>

// set values for horizontal link length, and retracted and extended angles
constexpr double LINK_LENGTH_METERS = 3.0 * 25.4;  // 3 inches in meters
constexpr double RETRACTED_ANGLE = 45.0;
constexpr double EXTENDED_ANGLE = -45.0;

// set up servo angle vector
double angles[4] = {RETRACTED_ANGLE, RETRACTED_ANGLE, RETRACTED_ANGLE, RETRACTED_ANGLE};

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

// // line between front pair of ToF and back pair
// double ToF_13[3] = {legs[2].x - legs[0].x, legs[2].y - legs[0].y, 0};
// double ToF_24[3] = {legs[3].x - legs[1].x, legs[4].y - legs[1].y, 0};

// reads range messages and saves them in correct array instance 
void rangeCallback(const sensor_msgs::Range::ConstPtr& msg, int index) {
    legs[index].z = -msg->range;  // terrain height: negative range
    legs[index].received = true;
}

// calculate leg positions for each leg
void calculatelegCommands() {
    // Make sure recieved values for all legs
    if (!std::all_of(legs.begin(), legs.end(), [](const leg_struct& l){ return l.received; }))
        return;

    // build plane
    // find vector between ToF 1 and 2 reading
    double v_12[3] = {legs[1].x - legs[0].x, legs[1].y - legs[0].y, legs[1].z - legs[0].z};
    // find vector between ToF 1 and 3
    double v_13[3] = {legs[2].x - legs[0].x, legs[2].y - legs[0].y, legs[2].z - legs[0].z};
    // find vector between ToF 2 and 4
    double v_24[3] = {legs[3].x - legs[1].x, legs[3].y - legs[1].y, legs[3].z - legs[1].z};
    // // cross product to find normal vector of plane
    // double n[3] = {v_12[1] * v_13[2] - v_12[2] * v_13[1], v_12[2] * v_13[0] - v_12[0] * v_13[2], v_12[0] * v_13[1] - v_12[1] * v_13[0]}; 
    
    // // project line between ToF sensors onto the plane
    // // calculate unit normal vector
    // double n_length = std::sqrt(n[0] * n[0] + n[1] * n[1] + n[2] * n[2]);
    // for (int i = 0; i < 3; ++i) {
    //     n[i] = n[i]/n_length;
    // }

    // // calculate projected vectors (vectors to match)
    // double v_proj_13[3];
    // double v_proj_24[3];
    // for (int i = 0; i < 3; ++i){
    //     v_proj_13[i] = ToF_13[i] - (ToF_13[0]*n[0] + ToF_13[1]*n[1] + ToF_13[2]*n[2])*n[i]; // legs 1 and 3
    //     v_proj_24[i] = ToF_24[i] - (ToF_24[0]*n[0] + ToF_24[1]*n[1] + ToF_24[2]*n[2])*n[i]; // legs 2 and 4        
    // }

    // // check if vectors can be matched
    // for(int i = 0; i < 4; ++1){
    //     if(i == 0 || i == 2){
    //         double y_check = (v_proj_13[1]/3);
    //         double check = v_proj_13[1]*v_proj_13[1] + (v_proj_13[2] + 4 + legs[i].z)^2;
    //     } else {
    //         double y_check = (v_proj_24[1]/3);
    //         double check = v_proj_24[1]*v_proj_24[1] + (v_proj_24[2] + 4 +  legs[i].z)^2;
    //     }
    //     if (check ~= 9 || y_check > 1 || y_check < -1) {
    //         ROS_WARN("Vectors cannot be matched, check leg positions and range data.");
    //         return;
    //     }
    // }

    // if check is succesful calculate angles for each leg and publish pwn commands
    for (int i = 0; i < 4; ++i){
        if(i == 0 || i == 2){
            angles[i] = atan(v_13[2]/(39.315*2)) * (180 / M_PI);
        } else {
            angles[i] = atan(v_24[2]/(39.315*2)) * (180 / M_PI);
        }
    }

    //flip angles for necessary legs
    angles[2] = -angles[2];
    angles[3] = -angles[3];

    for (int i =0; i<4; ++i){
    //clamp angles between 45 and -45
        angles[i] = std::clamp(angles[i], EXTENDED_ANGLE, RETRACTED_ANGLE);

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
            pwmMin + ((angles[i] - EXTENDED_ANGLE) / (RETRACTED_ANGLE - EXTENDED_ANGLE)) * (pwmMax - pwmMin));

        // create msg and publish
        std_msgs::UInt16 pwm_msg;
        pwm_msg.data = pwm;
        legs[i].pub.publish(pwm_msg);

        // print data to console for troubleshooting
        ROS_INFO_STREAM(std::fixed << std::setprecision(2)
            << "leg " << i+1
            << " | range = " << -legs[i].z << " mm"
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
