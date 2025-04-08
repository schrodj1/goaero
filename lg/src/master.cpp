#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include <iomanip>


double range_leg1 = 0.0;
double range_leg2 = 0.0;
double range_leg3 = 0.0;
double range_leg4 = 0.0;

bool received_leg1 = false;
bool received_leg2 = false;
bool received_leg3 = false;
bool received_leg4 = false;


// Callback function to process received range messages from each leg
void rangeCallbackLeg1(const sensor_msgs::Range::ConstPtr& msg) {
    range_leg1 = msg->range;
    received_leg1 = true;
}

void rangeCallbackLeg2(const sensor_msgs::Range::ConstPtr& msg) {
    range_leg2 = msg->range;
    received_leg2 = true;
}

void rangeCallbackLeg3(const sensor_msgs::Range::ConstPtr& msg) {
    range_leg3 = msg->range;
    received_leg3 = true;
}

void rangeCallbackLeg4(const sensor_msgs::Range::ConstPtr& msg) {
    range_leg4 = msg->range;
    received_leg4 = true;
}

void printRangeValues(){
if (received_leg1 && received_leg2 && received_leg3 && received_leg4){
	std::stringstream ss;
	
	ss<< std::fixed << std::setprecision(2);
	
	ss << "Leg 1: " << range_leg1 << "  "
		<< 	"Leg 2: " << range_leg2 << "  "
		<<	"Leg 3: " << range_leg3 << "  "
		<<	"Leg 4: " << range_leg4;
		
	ROS_INFO_STREAM(ss.str());
		
received_leg1 = received_leg2 = received_leg3 = received_leg4 = false;
	
}
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "master_node"); // Initialize the master node

    ros::NodeHandle n;

    // Subscribe to each of the four range topics
    ros::Subscriber sub_leg1 = n.subscribe("leg1/range", 1000, rangeCallbackLeg1);
    ros::Subscriber sub_leg2 = n.subscribe("leg2/range", 1000, rangeCallbackLeg2);
    ros::Subscriber sub_leg3 = n.subscribe("leg3/range", 1000, rangeCallbackLeg3);
    ros::Subscriber sub_leg4 = n.subscribe("leg4/range", 1000, rangeCallbackLeg4);

    std::cout << "Ready to receive.\n";
ros::Rate rate(1);
while (ros::ok()){
    ros::spinOnce(); // Keep the node running and processing callbacks
printRangeValues();
}
    return 0;
}
