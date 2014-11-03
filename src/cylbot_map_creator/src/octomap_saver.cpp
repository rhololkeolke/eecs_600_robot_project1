#include <ros/ros.h>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <string>
#include <fstream>

octomap_msgs::Octomap::ConstPtr octomap_msg;
ros::Subscriber octomap_sub;

void octomapCallback(const octomap_msgs::Octomap::ConstPtr& msg)
{
	octomap_msg = msg;
	octomap_sub.shutdown();
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "octomap_saver");

	ros::NodeHandle nh;

	ros::NodeHandle priv_nh("~");
	std::string output_file;
	priv_nh.param<std::string>("output_file", output_file, "~/octomap.bt");

	octomap_sub = nh.subscribe<octomap_msgs::Octomap>("/octomap", 1, &octomapCallback);

	ROS_INFO("Waiting for octomap");
	ros::Rate loop_rate(10);
	while(ros::ok() && octomap_msg == NULL)
	{
		ros::spinOnce();
		loop_rate.sleep();
	}

	if(!ros::ok())
	{
		ROS_INFO("Quitting");
		return 1;
	}

	ROS_INFO("Deserialzing octomap");
	
	octomap::AbstractOcTree* tree = octomap_msgs::msgToMap(*octomap_msg);
	octomap::OcTree* octree = dynamic_cast<octomap::OcTree*>(tree);
	if (octree == NULL)
	{
		ROS_ERROR("Failed to deserialize octomap");
		return 1;
	}

	ROS_INFO("Saving octomap to %s", output_file.c_str());

	std::ofstream file(output_file.c_str());
	octree->writeBinary(file);

	ROS_INFO("Success!");
}
