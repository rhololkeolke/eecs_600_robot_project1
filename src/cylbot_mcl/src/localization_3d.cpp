#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/passthrough.h>
#include <multisense_sensor_model/sensor_model.h>
#include <cylbot_mcl/pose_cloud3d.h>
#include <string>
#include <tf/transform_listener.h>
#include <boost/bind.hpp>
#include <boost/ref.hpp>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap_ros/conversions.h>
#include <octomap/octomap.h>



using namespace multisense_sensor_model;
using namespace cylbot_mcl;

ros::Subscriber octomap_sub;

void laserCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_in,
				   const tf::TransformListener& tf_listener,
				   PoseCloud3D* pose_cloud)
{
	tf::StampedTransform laser_transform;
	if(!tf_listener.waitForTransform(
		   "/map",
		   "/head_hokuyo_frame",
		   cloud_in->header.stamp,
		   ros::Duration(1.0)))
	{
		ROS_WARN("Transform from /map to /head_hokuyo_frame failed");
		return;
	}
	tf_listener.lookupTransform("/map", "/head_hokuyo_frame", cloud_in->header.stamp, laser_transform);
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
	if(!tf_listener.waitForTransform(
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
								 tf_listener);

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
	max_filter.setNegative(true);
	max_filter.setIndices(max_indices);
	max_filter.filter(*pcl_max_filtered_cloud);

	pcl_max_filtered_cloud->header.stamp = pcl_robot_filtered_cloud->header.stamp;
	pcl_max_filtered_cloud->header.frame_id = "/map";
	pcl::PointCloud<pcl::PointXYZ>::Ptr beam_ends(new pcl::PointCloud<pcl::PointXYZ>);
	if(!tf_listener.waitForTransform(
		   "/base_link",
		   "/map",
		   cloud_in->header.stamp,
		   ros::Duration(1.0)))
	{
		ROS_WARN("Transform from /base_link to /map failed");
		return;
	}
	pcl_ros::transformPointCloud("/base_link",
								 cloud_in->header.stamp,
								 *pcl_max_filtered_cloud,
								 "/map",
								 *beam_ends,
								 tf_listener);

	tf::StampedTransform map_transform;
	tf_listener.lookupTransform("/map", "/base_link", cloud_in->header.stamp, map_transform);

	geometry_msgs::Pose true_pose;
	tf::pointTFToMsg(map_transform.getOrigin(), true_pose.position);
	tf::quaternionTFToMsg(map_transform.getRotation(), true_pose.orientation);
	true_pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, tf::getYaw(true_pose.orientation) + 3.14159/2.0);

	pose_cloud->sensorUpdate(true_pose, *beam_ends, laser_transform.getOrigin(), cloud_in->header.stamp.toSec());
}

void initialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& initial_pose,
						 PoseCloud3D* pose_cloud,
						 const int num_particles)
{
	pose_cloud->resetCloud(*initial_pose, num_particles);
}

void velocityCallback(const geometry_msgs::TwistStamped::ConstPtr& twist_msg,
					  PoseCloud3D* pose_cloud,
					  geometry_msgs::TwistStamped::ConstPtr& last_cmd)
{
	if(last_cmd == NULL)
	{
		last_cmd = twist_msg;
		return;
	}
	ros::Duration dt = twist_msg->header.stamp - last_cmd->header.stamp;
	last_cmd = twist_msg;

	pose_cloud->motionUpdate(twist_msg->twist, dt.toSec());
}

void octomapCallback(const octomap_msgs::Octomap::ConstPtr& octomap_msg,
					 PoseCloud3D* pose_cloud)
{
	boost::shared_ptr<octomap::AbstractOcTree> tree(octomap_msgs::msgToMap(*octomap_msg));
	boost::shared_ptr<octomap::OcTree> octree = boost::dynamic_pointer_cast<octomap::OcTree>(tree);
	if(octree == NULL)
	{
		ROS_ERROR("Failed to deserialize octomap");
	}

	pose_cloud->octreeUpdate(octree);

	octomap_sub.shutdown();
}
					  

int main(int argc, char** argv)
{
	ros::init(argc, argv, "localization_2d");

	ros::NodeHandle nh;

	tf::TransformListener tf_listener;

	ros::NodeHandle priv_nh("~");
	std::string sensor_params_file = "";
	priv_nh.getParam("sensor_params_file", sensor_params_file);
	int num_global_samples;
	if(!priv_nh.getParam("num_global_samples", num_global_samples))
		num_global_samples = 10000;

	int num_local_samples;
	if(!priv_nh.getParam("num_local_samples", num_local_samples))
		num_local_samples = 1000;
	   

	IntrinsicParams sensor_params;
	if(sensor_params_file.compare("") == 0)
	{
		ROS_WARN("Sensor params file not provided or not valid");
	}
	else
	{
		readIntrinsicParamsFromFile(sensor_params_file, &sensor_params);
	}

	RobotModel model(sensor_params);
	
	for(int i=0; i<model.alpha.size(); i++)
	{
		model.alpha[i] = .01;
	}

	ROS_INFO("num_global_samples: %d num_local_samples %d", num_global_samples, num_local_samples);
	PoseCloud3D pose_cloud(model, num_global_samples);

	ros::Subscriber laser_sub = nh.subscribe<sensor_msgs::PointCloud2>("/laser/points", 1,
																	   boost::bind(laserCallback,
																				   _1,
																				   boost::ref(tf_listener),
																				   &pose_cloud));
	ros::Subscriber initial_pose_sub = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1,
																							  boost::bind(initialPoseCallback,
																										  _1,
																										  &pose_cloud,
																										  num_local_samples));

	geometry_msgs::TwistStamped::ConstPtr last_velocity;
	ros::Subscriber vel_sub = nh.subscribe<geometry_msgs::TwistStamped>("/cylbot/velocity", 1000,
																		boost::bind(velocityCallback,
																					_1,
																					&pose_cloud,
																					boost::ref(last_velocity)));


	octomap_sub = nh.subscribe<octomap_msgs::Octomap>("/octomap_binary", 1, boost::bind(octomapCallback, _1, &pose_cloud));

	ros::Publisher pose_array_pub = nh.advertise<geometry_msgs::PoseArray>("/pose_cloud", 1);

	ros::Rate loop_rate(100);
	while(ros::ok())
	{
		geometry_msgs::PoseArray pose_array = pose_cloud.getPoses();
		pose_array.header.stamp = ros::Time::now();
		pose_array.header.frame_id = "/map";
		pose_array_pub.publish(pose_array);
		
		ros::spinOnce();
		loop_rate.sleep();
	}
}
