#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap_ros/conversions.h>
#include <tf/transform_listener.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <vector>
#include <cmath>

octomap::OcTree* tree;
tf::TransformListener* tf_listener;
bool map_changed = true;


void laserPointsCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_in)
{
	tf::StampedTransform laser_transform;
	if(!tf_listener->waitForTransform(
		   "/map",
		   "/head_hokuyo_frame",
		   cloud_in->header.stamp,
		   ros::Duration(1.0)))
	{
		ROS_WARN("Transform from /map to /head_hokuyo_frame failed");
		return;
	}
	tf_listener->lookupTransform("/map", "/head_hokuyo_frame", cloud_in->header.stamp, laser_transform);
	octomap::point3d laser_origin = octomap::pointTfToOctomap(laser_transform.getOrigin());

	pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_raw_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::fromROSMsg(*cloud_in, *pcl_raw_cloud);

    // filter out the points on the robot
	pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_robot_filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);

	pcl::ConditionOr<pcl::PointXYZ>::Ptr range_cond (new pcl::ConditionOr<pcl::PointXYZ>());
	range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (
								  new pcl::FieldComparison<pcl::PointXYZ>("x", pcl::ComparisonOps::GE, 0.25)));
	range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (
								  new pcl::FieldComparison<pcl::PointXYZ>("x", pcl::ComparisonOps::LE, -0.25)));
	range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (
								  new pcl::FieldComparison<pcl::PointXYZ>("y", pcl::ComparisonOps::GE, 0.25)));
	range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (
								  new pcl::FieldComparison<pcl::PointXYZ>("y", pcl::ComparisonOps::LE, -0.25)));
	range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (
								  new pcl::FieldComparison<pcl::PointXYZ>("z", pcl::ComparisonOps::GE, 1.2)));
	range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (
								  new pcl::FieldComparison<pcl::PointXYZ>("z", pcl::ComparisonOps::LE, -.1)));
	pcl::ConditionalRemoval<pcl::PointXYZ> robot_filter(range_cond);
	robot_filter.setInputCloud(pcl_raw_cloud);
	robot_filter.setKeepOrganized(true);
	robot_filter.filter(*pcl_robot_filtered_cloud);

	ROS_DEBUG("pcl_robot_filtered_cloud size: %lu", pcl_robot_filtered_cloud->points.size());
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_map_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	if(!tf_listener->waitForTransform(
		   "/map",
		   "/base_link",
		   cloud_in->header.stamp,
		   ros::Duration(1.0)))
	{
		ROS_WARN("Transform from /map to /base_link failed");
		return;
	}
	pcl_ros::transformPointCloud("/map",
								 cloud_in->header.stamp,
								 *pcl_robot_filtered_cloud,
								 "/base_link",
								 *pcl_map_cloud,
								 *tf_listener);

	// filter out the ground
	pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_ground_filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PassThrough<pcl::PointXYZ> ground_filter;
	ground_filter.setInputCloud(pcl_map_cloud);
	ground_filter.setFilterFieldName("z");
	ground_filter.setFilterLimits(-1, .05);
	ground_filter.setFilterLimitsNegative(true);
	ground_filter.filter(*pcl_ground_filtered_cloud);

	ROS_DEBUG("pcl_ground_filtered_cloud size: %lu", pcl_ground_filtered_cloud->points.size());

	// filter out the max range readings
	pcl::PointIndices::Ptr max_indices(new pcl::PointIndices);
	int i=0;
	for(pcl::PointCloud<pcl::PointXYZ>::const_iterator point = pcl_ground_filtered_cloud->points.begin();
		point != pcl_ground_filtered_cloud->points.end();
		point++)
	{
		// if this point is within .03 m of the max sensor reading then we want to remove it
		double x = point->x - laser_origin.x();
		double y = point->y - laser_origin.y();
		double z = point->z - laser_origin.z();
		if(fabs(sqrt(x*x + y*y + z*z) - 30.0) < .04)
			max_indices->indices.push_back(i);
		i++;
	}
	pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_max_filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::ExtractIndices<pcl::PointXYZ> max_filter;
	max_filter.setInputCloud(pcl_ground_filtered_cloud);
	max_filter.setKeepOrganized(true);
	max_filter.setNegative(true);
	max_filter.setIndices(max_indices);
	max_filter.filter(*pcl_max_filtered_cloud);

	ROS_DEBUG("pcl_max_filtered_cloud size: %lu", pcl_max_filtered_cloud->points.size());

	// convert to ros datatype as an intermediate step to converting to octomap type
	sensor_msgs::PointCloud2 filtered_cloud;
	pcl::toROSMsg(*pcl_max_filtered_cloud, filtered_cloud);

	ROS_DEBUG("filtered_cloud size: %lu", filtered_cloud.data.size());

	// convert to octomap type
	octomap::Pointcloud octomap_cloud;
	octomap::pointCloud2ToOctomap(filtered_cloud, octomap_cloud);

	ROS_DEBUG("octomap pointcloud size: %lu", octomap_cloud.size());

	tree->insertPointCloudRays(octomap_cloud, laser_origin, -1, true);

	ROS_DEBUG("map_changed is now true");
	map_changed = true;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "octomap_creator");

	ros::NodeHandle nh;

	tf_listener = new tf::TransformListener();
	ros::Publisher occupied_pub = nh.advertise<octomap_msgs::Octomap>("/octomap", 1, true);
	
	ros::NodeHandle priv_nh("~");
	double resolution;
	priv_nh.param<double>("resolution", resolution, 0.1);

	ROS_INFO("Creating tree with resolution %3.3f", resolution);

	tree = new octomap::OcTree(resolution);

	ros::Subscriber laser_sub = nh.subscribe<sensor_msgs::PointCloud2>("/laser/points", 100, &laserPointsCallback);

	octomap_msgs::Octomap octomap_msg;
	octomap_msg.header.frame_id = "/map";

	ros::Rate loop_rate(1);
	while(ros::ok())
	{
		if(map_changed)
		{
			ROS_DEBUG("preparing to publish data");
			ROS_DEBUG("tree size: %lu", tree->size());
			// this is necessary because the callback uses lazy evaluation
			tree->updateInnerOccupancy();
			
			// convert to the ros message type
			octomap_msg.data.clear();
			octomap_msgs::fullMapToMsg(*tree, octomap_msg);

			if(octomap_msg.data.size() > 0)
			{
				ROS_DEBUG("Publishing octomap");
				octomap_msg.header.stamp = ros::Time::now();
				occupied_pub.publish(octomap_msg);
			}
			map_changed = false;
		}
		
		ros::spinOnce();
		loop_rate.sleep();
	}

	delete tf_listener;
	delete tree;
}
