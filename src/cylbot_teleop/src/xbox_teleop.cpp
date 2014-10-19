#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>

#define RIGHT_STICK_LR (3)
#define RIGHT_STICK_UD (4)

ros::Publisher cmd_vel_pub;

void joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
	geometry_msgs::Twist cmd_msg;
	cmd_msg.linear.x = joy->axes[RIGHT_STICK_UD];
	cmd_msg.angular.z = -joy->axes[RIGHT_STICK_LR];

	cmd_vel_pub.publish(cmd_msg);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "cylbot_xbox_teleop");

	ros::NodeHandle nh;
	cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("cylbot/cmd_vel", 1);

	ros::Subscriber joy_sub = nh.subscribe<sensor_msgs::Joy>("joy", 10, &joyCallback);
	
	ros::spin();
}
