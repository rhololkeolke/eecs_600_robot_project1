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

	OcTree tree(0.1);

	for (int x=-20; x<20; x++) {
		for (int y=-20; y<20; y++) {
			for (int z=-20; z<20; z++) {
				point3d endpoint ((float) x*0.05f, (float) y*0.05f, (float) z*0.05f);
				tree.updateNode(endpoint, true); // integrate 'occupied' measurement
			}
		}
	}
	
	for (int x=-30; x<30; x++) {
		for (int y=-30; y<30; y++) {
			for (int z=-30; z<30; z++) {
				point3d endpoint ((float) x*0.02f-1.0f, (float) y*0.02f-1.0f, (float) z*0.02f-1.0f);
				tree.updateNode(endpoint, false);  // integrate 'free' measurement
			}
		}
	}

	ros::NodeHandle nh;

	ros::Publisher occupied_pub = nh.advertise<octomap_msgs::Octomap>("/octomap", 1, true);

	octomap_msgs::Octomap octomap_msg;
	octomap_msg.header.stamp = ros::Time::now();
	octomap_msg.header.frame_id = "/map";
	octomap_msgs::fullMapToMsg(tree, octomap_msg);

	occupied_pub.publish(octomap_msg);

	ros::spin();
}
