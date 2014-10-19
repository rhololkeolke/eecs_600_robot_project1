#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <boost/thread/mutex.hpp>

#define RIGHT_STICK_LR (3)
#define RIGHT_STICK_UD (4)

#define MAX_DELTA_LINEAR (.5)
#define MAX_DELTA_ANGULAR (.5)

boost::mutex set_point;
double linear=0, angular=0;

void joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
	boost::mutex::scoped_lock lock(set_point);
	
	linear = joy->axes[RIGHT_STICK_UD];
	angular = -joy->axes[RIGHT_STICK_LR];
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "cylbot_xbox_teleop");

	ros::NodeHandle nh;
	ros::Publisher cmd_pub = nh.advertise<geometry_msgs::Twist>("cylbot/cmd_vel", 1);

	ros::Subscriber joy_sub = nh.subscribe<sensor_msgs::Joy>("joy", 10, &joyCallback);

	geometry_msgs::Twist cmd_msg;
	double p = .05;

	double dt = 1/100.0;
	ros::Rate loop_rate(100.0);
	while(ros::ok())
	{
		{
			boost::mutex::scoped_lock lock(set_point);

			double delta_linear = linear - cmd_msg.linear.x;
			double delta_angular = angular - cmd_msg.angular.z;

			// saturate the linear command
			if(delta_linear/dt > MAX_DELTA_LINEAR)
				delta_linear = MAX_DELTA_LINEAR*dt;
			else if(delta_linear/dt < -MAX_DELTA_LINEAR)
				delta_linear = -MAX_DELTA_LINEAR*dt;

			// saturate the angular command
			if(delta_angular/dt > MAX_DELTA_ANGULAR)
				delta_angular = MAX_DELTA_ANGULAR*dt;
			else if(delta_angular/dt < -MAX_DELTA_ANGULAR)
				delta_angular = -MAX_DELTA_ANGULAR*dt;

			cmd_msg.linear.x += delta_linear;
			cmd_msg.angular.z += delta_angular;
		}

		cmd_pub.publish(cmd_msg);
		
		ros::spinOnce();
		loop_rate.sleep();
	}
}
