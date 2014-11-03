#include <ros/ros.h>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>

using namespace std;
using namespace octomap;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "octree_to_rviz_test");

	ros::NodeHandle nh;
	
	OcTree tree(0.1);

	point3d origin ((float) 0, (float) 0, (float) 0);
	point3d x_axis (5.0f, 0.0f, 0.0f);
	point3d y_axis (0.0f, 5.0f, 0.0f);
	point3d z_axis (0.0f, 0.0f, 5.0f);

	tree.insertRay(origin, x_axis);
	tree.insertRay(origin, y_axis);
	tree.insertRay(origin, z_axis);

	point3d hitPoint;
	if(tree.castRay(origin, x_axis, hitPoint, true, 10.0))
	{
		ROS_INFO("Ray cast to 5.0 hit at (%3.3f, %3.3f, %3.3f)", hitPoint.x(), hitPoint.y(), hitPoint.z());
	}
	else
	{
		ROS_INFO("Ray cast to 5.0 hit nothing");
	}
	
	x_axis.x() = 1.0f;
	if(tree.castRay(origin, x_axis, hitPoint, true, 1.0))
	{
		ROS_INFO("Ray cast to 1.0 hit at (%3.3f, %3.3f, %3.3f)", hitPoint.x(), hitPoint.y(), hitPoint.z());
	}
	else
	{
		ROS_INFO("Ray cast to 1.0 hit nothing");
	}

	x_axis.x() = 10.0f;
	if(tree.castRay(origin, x_axis, hitPoint, true, 10.0))
	{
		ROS_INFO("Ray cast to 10.0 hit at (%3.3f, %3.3f, %3.3f)", hitPoint.x(), hitPoint.y(), hitPoint.z());
	}
	else
	{
		ROS_INFO("Ray cast to 10.0 hit nothing");
	}


	ros::Publisher occupied_pub = nh.advertise<octomap_msgs::Octomap>("/octomap", 1, true);

	octomap_msgs::Octomap octomap_msg;
	octomap_msg.header.stamp = ros::Time::now();
	octomap_msg.header.frame_id = "/map";
	octomap_msgs::fullMapToMsg(tree, octomap_msg);

	occupied_pub.publish(octomap_msg);

	ros::spin();
}
