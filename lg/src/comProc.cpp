#include <ros/ros.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Float32.h>

double angle_leg1 = 0.0, angle_leg2 = 0.0, angle_leg3 = 0.0, angle_leg4 = 0.0;

ros::Publisher pwm_pub_leg1, pwm_pub_leg2, pwm_pub_leg3, pwm_pub_leg4;

int pwmMap(double input) {
    double inputMin = 0, inputMax = 115;
    uint8_t outputMin = 35, outputMax = 150;

    if (inputMax == inputMin) return outputMin;  // Prevent division by zero

    return static_cast<int>(outputMin + ((input - inputMin) / (inputMax - inputMin)) * (outputMax - outputMin));
}

void rangeCallback(const std_msgs::Float32::ConstPtr& msg, ros::Publisher& pwm_pub) {
    std_msgs::UInt8 pwm_msg;
    pwm_msg.data = pwmMap(msg->data);
    pwm_pub.publish(pwm_msg);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "comProc");
    ros::NodeHandle n;

    // Publishers for PWM commands
    pwm_pub_leg1 = n.advertise<std_msgs::UInt8>("leg1/pwm", 10);
    pwm_pub_leg2 = n.advertise<std_msgs::UInt8>("leg2/pwm", 10);
    pwm_pub_leg3 = n.advertise<std_msgs::UInt8>("leg3/pwm", 10);
    pwm_pub_leg4 = n.advertise<std_msgs::UInt8>("leg4/pwm", 10);

    // Subscribers using std::bind for a cleaner approach
    ros::Subscriber sub_leg1 = n.subscribe<std_msgs::Float32>("leg1/numAngle", 1000, 
        boost::bind(rangeCallback, _1, boost::ref(pwm_pub_leg1)));
    ros::Subscriber sub_leg2 = n.subscribe<std_msgs::Float32>("leg2/numAngle", 1000, 
        boost::bind(rangeCallback, _1, boost::ref(pwm_pub_leg2)));
    ros::Subscriber sub_leg3 = n.subscribe<std_msgs::Float32>("leg3/numAngle", 1000, 
        boost::bind(rangeCallback, _1, boost::ref(pwm_pub_leg3)));
    ros::Subscriber sub_leg4 = n.subscribe<std_msgs::Float32>("leg4/numAngle", 1000, 
        boost::bind(rangeCallback, _1, boost::ref(pwm_pub_leg4)));

    ros::spin();  // Blocking call to handle callbacks indefinitely
    return 0;
}