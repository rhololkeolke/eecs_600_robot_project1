#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv) {
	ros::init(argc, argv, "state_publisher");
	ros::NodeHandle n;
	ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);
	ros::Rate loop_rate(30);

	sensor_msgs::JointState joint_state;
	
	while(ros::ok()) {

		
		loop_rate.sleep();
	}
}
