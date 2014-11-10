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
#include <geometry_msgs/Pose.h>

using namespace multisense_sensor_model;
using namespace cylbot_mcl;

ros::Subscriber field_sub;

void laserCallback(const sensor_msgs::PointCloud2::ConstPtr& laser_points,
				   const tf::TransformListener& tf_listener,
				   PoseCloud2D* pose_cloud)
{
	tf::StampedTransform laser_transform;
	if(!tf_listener.waitForTransform(
		   "/base_link",
		   "/head_hokuyo_frame",
		   laser_points->header.stamp,
		   ros::Duration(1.0)))
	{
		ROS_WARN("Transform from /base_link to /head_hokuyo_frame failed");
		return;
	}

	tf_listener.lookupTransform("/base_link", "/head_hokuyo_frame", laser_points->header.stamp, laser_transform);
	tf::Vector3 beam_start = laser_transform.getOrigin();

	pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_raw_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::fromROSMsg(*laser_points, *pcl_raw_cloud);

	pcl::PointIndices::Ptr invalid_indices(new pcl::PointIndices);
	int i=0;
	for(pcl::PointCloud<pcl::PointXYZ>::const_iterator point = pcl_raw_cloud->points.begin();
		point != pcl_raw_cloud->points.end();
		point++)
	{
		if(point->x != point->x || point->y != point->y || point->z != point->z)
		{
			ROS_WARN("Point has a NaN value");
		}

		// calculate 3D beam length and check for max value
		double x = point->x - beam_start.x();
		double y = point->y - beam_start.y();
		double z = point->z - beam_start.z();
		double beam_length_3d = sqrt(x*x + y*y + z*z);
		if(beam_length_3d > 30.0)
		{
			invalid_indices->indices.push_back(i);
			i++;
			continue;
		}

		// check if point is in the plane (or at least close to it)
		if(point->z > .8034 || point->z < .4034)
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
	invalid_filter.setNegative(true);
	invalid_filter.setIndices(invalid_indices);
	invalid_filter.filter(*pcl_cloud);



	tf::StampedTransform map_transform;
	if(!tf_listener.waitForTransform(
		   "/map",
		   "/base_link",
		   laser_points->header.stamp,
		   ros::Duration(1.0)))
	{
		ROS_WARN("Transform from /map to /base_link failed");
		return;
	}
	tf_listener.lookupTransform("/map", "/base_link", laser_points->header.stamp, map_transform);

	geometry_msgs::Pose true_pose;
	tf::pointTFToMsg(map_transform.getOrigin(), true_pose.position);
	tf::quaternionTFToMsg(map_transform.getRotation(), true_pose.orientation);

	pose_cloud->sensorUpdate(true_pose, *pcl_cloud, laser_points->header.stamp.toSec());

	//ROS_DEBUG("Starting true pose probability calculation");
	//double truePoseProb = pose_cloud->getMeasurementProbability(true_pose, true_pose, *pcl_cloud);
	//ROS_DEBUG_STREAM("true pose probability:" << truePoseProb);

	geometry_msgs::Pose fake_pose = true_pose;
	fake_pose.position.x *= 2.1;
	fake_pose.position.y *= 2.1;
	//ROS_DEBUG("Starting fake pose probability calculation");
	//double fakePoseProb = pose_cloud->getMeasurementProbability(true_pose, fake_pose, *pcl_cloud);
	//ROS_DEBUG_STREAM("fake pose probability:" << fakePoseProb);
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

void fieldCallback(const cylbot_map_creator::LikelihoodField::ConstPtr& field,
				 PoseCloud2D* pose_cloud)
{
	pose_cloud->fieldUpdate(*field);

	field_sub.shutdown();
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
	
	for(int i=0; i<model.alpha.size(); i++)
	{
		model.alpha[i] = .01;
	}
	
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
	ros::Subscriber vel_sub = nh.subscribe<geometry_msgs::TwistStamped>("/cylbot/velocity", 100,
																		boost::bind(velocityCallback,
																					_1,
																					&pose_cloud,
																					boost::ref(last_velocity)));

	field_sub = nh.subscribe<cylbot_map_creator::LikelihoodField>("/likelihood_field", 1, boost::bind(fieldCallback, _1, &pose_cloud));

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
