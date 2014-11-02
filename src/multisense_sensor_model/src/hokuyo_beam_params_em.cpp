#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Pose.h>
#include <visualization_msgs/Marker.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <tf/transform_listener.h>
#include <pcl_ros/transforms.h>
#include <boost/math/distributions/exponential.hpp>
#include <boost/math/distributions/normal.hpp>
#include <boost/math/distributions/uniform.hpp>
#include <boost/shared_ptr.hpp>
#include <vector>
#include <string>

typedef struct IntrinsicParams_ {
	double zhit, zshort, zmax, zrand;
	double sigma_hit;
	double lambda_short;
} IntrinsicParams;

typedef struct ScanData_ {
	pcl::PointCloud<pcl::PointXYZ> beam_ends;
	tf::Vector3 beam_start;
} ScanData;

boost::shared_ptr<tf::TransformListener> tf_listener;
ros::Subscriber laser_sub;
ros::Subscriber wall_info_sub;

int num_scans_to_collect = 10;
std::vector<boost::shared_ptr<ScanData> > laser_scans;

geometry_msgs::Pose wall_pose;
geometry_msgs::Vector3 wall_scale;

IntrinsicParams learnIntrinsicParams(const std::vector<boost::shared_ptr<ScanData> > laser_scans);

// this method just collects data
void laserScanCallback(const sensor_msgs::PointCloud2::ConstPtr& laser_points)
{
	if(laser_scans.size() < num_scans_to_collect)
	{
		boost::shared_ptr<ScanData> scan_data(new ScanData());
		sensor_msgs::PointCloud2 map_laser_points;
		if(!tf_listener->waitForTransform(
			   "/map",
			   laser_points->header.frame_id,
			   laser_points->header.stamp,
			   ros::Duration(1.0)))
		{
			ROS_WARN("Failure to get transform from /map to %s at %3.3f",
					 laser_points->header.frame_id.c_str(),
					 laser_points->header.stamp.toSec());
			return;
		}
		pcl_ros::transformPointCloud("/map", *laser_points, map_laser_points, *tf_listener);
			   
		// the data is only useful if there is an associated pose
		// so wait for a transform and if there is none then ignore
		// this piece of data and warn the user
		if(!tf_listener->waitForTransform(
			   "/map",
			   "/head_hokuyo_frame",
			   laser_points->header.stamp,
			   ros::Duration(1.0)))
		{
			ROS_WARN("Failure to get transform from /map to /head_hokuyo_frame at %3.3f",
					 laser_points->header.stamp.toSec());
			return;
		}
		tf::StampedTransform laser_transform;
		tf_listener->lookupTransform("/map", "/head_hokuyo_frame", laser_points->header.stamp, laser_transform);
		scan_data->beam_start = laser_transform.getOrigin();

		pcl::fromROSMsg(map_laser_points, scan_data->beam_ends);

		// if this is the last piece of data print a helpful message
		// and stop running this subscriber
		if(laser_scans.size() == num_scans_to_collect - 1)
		{
			ROS_INFO("Finished collecting data unsubscribing and processing data");
			laser_sub.shutdown();
		}

		laser_scans.push_back(scan_data);
	}
}

void wallCallback(const visualization_msgs::Marker::ConstPtr& wall_info)
{
	ROS_INFO("Got wall info");
	ROS_INFO_STREAM(*wall_info);
	wall_pose = wall_info->pose;
	wall_scale = wall_info->scale;

	wall_info_sub.shutdown();
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "hokuyo_beam_params_em");

	ros::NodeHandle nh;
	tf_listener.reset(new tf::TransformListener());

	ros::NodeHandle priv_nh("~");
	priv_nh.getParam("num_scans_to_collect", num_scans_to_collect);

	ROS_INFO("Collecting %d scans", num_scans_to_collect);
	
	laser_sub = nh.subscribe<sensor_msgs::PointCloud2>("/laser/points", 10, &laserScanCallback);
	wall_info_sub = nh.subscribe<visualization_msgs::Marker>("/gazebo/unit_box_1/info", 10, &wallCallback);
	
	ros::Rate loop_rate(100);
	while(ros::ok() && laser_scans.size() < num_scans_to_collect)
	{
		ros::spinOnce();
		loop_rate.sleep();
	}

	// if the loop exited because the node is shuttingdown
	// then don't run the EM algorithm
	if(!ros::ok())
		return 0;

	ROS_INFO("Collected %lu scans. Starting EM", laser_scans.size());

	IntrinsicParams params = learnIntrinsicParams(laser_scans);
}

IntrinsicParams learnIntrinsicParams(const std::vector<boost::shared_ptr<ScanData> > laser_scans)
{
	IntrinsicParams params;
	return params;
}
