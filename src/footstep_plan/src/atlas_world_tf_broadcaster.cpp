#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <boost/bind.hpp>


void odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg,
				  tf::TransformBroadcaster* tf_broadcaster)
{
	tf::Transform transform;
	transform.setOrigin(tf::Vector3(odom_msg->pose.pose.position.x,
									odom_msg->pose.pose.position.y,
									odom_msg->pose.pose.position.z));
	transform.setRotation(tf::Quaternion(odom_msg->pose.pose.orientation.x,
										 odom_msg->pose.pose.orientation.y,
										 odom_msg->pose.pose.orientation.z,
										 odom_msg->pose.pose.orientation.w));
	tf_broadcaster->sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "pelvis"));
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "atlas_world_tf_broadcaster");
	ros::NodeHandle node;

	tf::TransformBroadcaster tf_broadcaster;
	
	ros::Subscriber sub = node.subscribe<nav_msgs::Odometry>("/ground_truth_odom",
															 10,
															 boost::bind(odomCallback, _1, &tf_broadcaster));

	ros::spin();
}
