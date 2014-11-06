#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <multisense_sensor_model/sensor_model.h>
#include <cylbot_mcl/pose_cloud.h>
#include <string>
#include <tf/transform_listener.h>
#include <boost/bind.hpp>
#include <boost/ref.hpp>
#include <pcl/filters/extract_indices.h>

using namespace multisense_sensor_model;
using namespace cylbot_mcl;

void laserCallback(const sensor_msgs::PointCloud2::ConstPtr& laser_points,
				   const tf::TransformListener& tf_listener,
				   PoseCloud2D* pose_cloud)
{
	sensor_msgs::PointCloud2 map_ros_cloud;
	if(!tf_listener.waitForTransform(
		   "/map",
		   "/base_link",
		   laser_points->header.stamp,
		   ros::Duration(1.0)))
	{
		ROS_WARN("Transform from /map to /base_link failed");
		return;
	}
	pcl_ros::transformPointCloud("/map", *laser_points, map_ros_cloud, tf_listener);

	tf::StampedTransform laser_transform;
	if(!tf_listener.waitForTransform(
		   "/map",
		   "/head_hokuyo_frame",
		   laser_points->header.stamp,
		   ros::Duration(1.0)))
	{
		ROS_WARN("Transform from /map to /head_hokuyo_frame failed");
		return;
	}

	tf_listener.lookupTransform("/map", "/head_hokuyo_frame", laser_points->header.stamp, laser_transform);
	tf::Vector3 beam_start = laser_transform.getOrigin();

	pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_raw_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::fromROSMsg(map_ros_cloud, *pcl_raw_cloud);

	pcl::PointIndices::Ptr invalid_indices(new pcl::PointIndices);
	int i=0;
	for(pcl::PointCloud<pcl::PointXYZ>::const_iterator point = pcl_raw_cloud->points.begin();
		point != pcl_raw_cloud->points.end();
		point++)
	{
		// calculate 3D beam length and check for max value
		double x = point->x - beam_start.x();
		double y = point->y - beam_start.y();
		double z = point->z - beam_start.z();
		double beam_length_3d = sqrt(x*x + y*y + z*z);
		if(fabs(beam_length_3d - 30.0) < .04)
		{
			invalid_indices->indices.push_back(i);
			i++;
			continue;
		}

		// check if point is in the plane (or at least close to it)
		if(point->z > 1.2 || point->z < .8)
		{
			invalid_indices->indices.push_back(i);
			i++;
			continue;
		}

		// check if the beam is within the robot's cylinder
		double beam_length_2d = sqrt(x*x + y*y);
		if(fabs(beam_length_2d ) < .5)
		{
			invalid_indices->indices.push_back(i);
			i++;
			continue;
		}
	}

	pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::ExtractIndices<pcl::PointXYZ> invalid_filter;
	invalid_filter.setInputCloud(pcl_raw_cloud);
	invalid_filter.setKeepOrganized(true);
	invalid_filter.setNegative(true);
	invalid_filter.setIndices(invalid_indices);
	invalid_filter.filter(*pcl_cloud);

	beam_start.setZ(0);
	
	pose_cloud->sensorUpdate(*pcl_cloud, beam_start);
}

void initialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& initial_pose,
						 PoseCloud2D* pose_cloud,
						 const int num_particles)
{
	pose_cloud->resetCloud(*initial_pose, num_particles);
}

void velocityCallback(const geometry_msgs::TwistStamped::ConstPtr& twist_msg,
					  PoseCloud2D* pose_cloud,
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

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& map,
				 PoseCloud2D* pose_cloud)
{
	pose_cloud->mapUpdate(*map);
}
					  

int main(int argc, char** argv)
{
	ros::init(argc, argv, "localization_2d");

	ros::NodeHandle nh;

	tf::TransformListener tf_listener;

	ros::NodeHandle priv_nh("~");
	std::string sensor_params_file = "";
	priv_nh.getParam("sensor_params_file", sensor_params_file);

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
	PoseCloud2D pose_cloud(model);

	ros::Subscriber laser_sub = nh.subscribe<sensor_msgs::PointCloud2>("/laser/points", 1,
																	   boost::bind(laserCallback,
																				   _1,
																				   boost::ref(tf_listener),
																				   &pose_cloud));
	ros::Subscriber initial_pose_sub = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1,
																							  boost::bind(initialPoseCallback,
																										  _1,
																										  &pose_cloud,
																										  1000));

	geometry_msgs::TwistStamped::ConstPtr last_velocity;
	ros::Subscriber vel_sub = nh.subscribe<geometry_msgs::TwistStamped>("/cylbot/velocity", 1,
																		boost::bind(velocityCallback,
																					_1,
																					&pose_cloud,
																					boost::ref(last_velocity)));

	ros::Subscriber map_sub = nh.subscribe<nav_msgs::OccupancyGrid>("/map", 1, boost::bind(mapCallback, _1, &pose_cloud));

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
