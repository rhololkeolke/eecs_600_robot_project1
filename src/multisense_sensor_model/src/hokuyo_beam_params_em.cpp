#include <ros/ros.h>
#include "hokuyo_beam_params_em.h"
#include <sensor_msgs/PointCloud2.h>

#include <visualization_msgs/Marker.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf/transform_listener.h>
#include <pcl_ros/transforms.h>
#include <string>

using namespace multisense_sensor_model;

boost::shared_ptr<tf::TransformListener> tf_listener;
ros::Subscriber laser_sub;
ros::Subscriber wall_info_sub;

int num_scans_to_collect = 10;
ScanDataVector laser_scans;

geometry_msgs::Pose wall_pose;
geometry_msgs::Vector3 wall_scale;

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
	ROS_DEBUG_STREAM(*wall_info);
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

	IntrinsicParams params = learnIntrinsicParams(laser_scans, .01, 1.0);

	printIntrinsicParams(params);
}

IntrinsicParams learnIntrinsicParams(const ScanDataVector laser_scans,
									 const double sigma_hit, const double lambda_short,
									 const double epsilon, const int max_iterations)
{

	IntrinsicParams old_params;
	IntrinsicParams new_params;
	new_params.zhit = new_params.zshort = new_params.zmax = new_params.zrand = 0.25;
	new_params.sigma_hit = sigma_hit;
	new_params.lambda_short = lambda_short;

	int iteration = 0;
	do {
		old_params = new_params;

		int Zcard = 0;
		double total_ehit=0, total_eshort=0, total_emax=0, total_erand=0;
		double sigma_update_inner_sum = 0;
		double lambda_update_denom = 0;
		for(ScanDataVector::const_iterator scan = laser_scans.begin();
			scan != laser_scans.end();
			scan++)
		{
			for(pcl::PointCloud<pcl::PointXYZ>::const_iterator beam_end = (*scan)->beam_ends.begin();
				beam_end != (*scan)->beam_ends.end();
				beam_end++)
			{
				tf::Vector3 beam_end_vec(beam_end->x, beam_end->y, beam_end->z);
				double mapDist = distanceAccordingToMap((*scan)->beam_start, beam_end_vec, wall_pose, wall_scale);

				if(mapDist == -1)
					mapDist = 30.0;

				tf::Vector3 beam_vec = beam_end_vec - (*scan)->beam_start;
				double beam_length = sqrt(beam_vec.getX()*beam_vec.getX() +
										  beam_vec.getY()*beam_vec.getY() +
										  beam_vec.getZ()*beam_vec.getZ());

				double eta = 1.0/(old_params.pHit(beam_length, mapDist) +
								  old_params.pShort(beam_length, mapDist) +
								  old_params.pMax(beam_length) +
								  old_params.pRand(beam_length));

				// ROS_INFO("%3.3f, %3.3f -- %3.3f -- %3.3f, %3.3f, %3.3f, %3.3f", mapDist, beam_length, eta, e_hit, e_short, e_max, e_rand);
				double ehit = eta*old_params.pHit(beam_length, mapDist);
				sigma_update_inner_sum += ehit*(beam_length - mapDist)*(beam_length - mapDist);
				total_ehit += ehit;

				double eshort = eta*old_params.pShort(beam_length, mapDist);
				lambda_update_denom += eshort*beam_length;
				total_eshort += eshort;

				double emax = eta*old_params.pMax(beam_length);
				total_emax += emax;
				double erand = eta*old_params.pRand(beam_length);
				total_erand += erand;

				Zcard++;
				
			}
		}

		double ZcardInv = 1.0/((double)Zcard);

		// update the parameters
		new_params.zhit = ZcardInv*total_ehit;
		new_params.zshort = ZcardInv*total_eshort;
		new_params.zmax = ZcardInv*total_emax;
		new_params.zrand = ZcardInv*total_erand;

		new_params.sigma_hit = sqrt(sigma_update_inner_sum/total_ehit);
		new_params.lambda_short = total_eshort/lambda_update_denom;

		iteration++;
		ROS_DEBUG("Iteration: %d of %d with change of %3.3f",
				  iteration,
				  max_iterations,
				  getParameterDelta(new_params, old_params));
		//printIntrinsicParams(new_params);
	} while(getParameterDelta(new_params, old_params) > epsilon && iteration < max_iterations);

	if(iteration < max_iterations)
	{
		ROS_INFO("EM algorithm converged");
		new_params.converged = true;
	}
	else
	{
		ROS_WARN("EM algorithm failed to converge after %d iterations", max_iterations);
	}

	return new_params;
}
