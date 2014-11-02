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
#include <cmath>

typedef struct IntrinsicParams_ {
	double zhit, zshort, zmax, zrand;
	double sigma_hit;
	double lambda_short;
} IntrinsicParams;

typedef struct ScanData_ {
	pcl::PointCloud<pcl::PointXYZ> beam_ends;
	tf::Vector3 beam_start;
} ScanData;

typedef std::vector<boost::shared_ptr<ScanData> > ScanDataVector;

boost::shared_ptr<tf::TransformListener> tf_listener;
ros::Subscriber laser_sub;
ros::Subscriber wall_info_sub;

int num_scans_to_collect = 10;
ScanDataVector laser_scans;

geometry_msgs::Pose wall_pose;
geometry_msgs::Vector3 wall_scale;

IntrinsicParams learnIntrinsicParams(const ScanDataVector laser_scans,
									 const double sigma_hit, const double lambda_short,
									 const double epsilon=1e-5, const int max_iterations=20);

double distanceAccordingToMap(tf::Vector3 beam_start, tf::Vector3 beam_end);
double getPMax(const double sensor_value, const double epsilon=.01)
{
	if(fabs(sensor_value - 30.0) > epsilon)
		return 0.0;
	return 1.0;
}

void printIntrinsicParams(const IntrinsicParams& params)
{
	ROS_INFO("zhit: %3.3f\nzshort: %3.3f\nzmax: %3.3f\nzrand: %3.3f\nsigma_hit: %3.3f\nlambda_short: %3.3f",
			 params.zhit,
			 params.zshort,
			 params.zmax,
			 params.zrand,
			 params.sigma_hit,
			 params.lambda_short);
}

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
}

double getParameterDelta(const IntrinsicParams& params1, const IntrinsicParams& params2)
{
	double result = 0;
	result += fabs(params1.zhit - params2.zhit);
	result += fabs(params1.zshort - params2.zshort);
	result += fabs(params1.zmax - params2.zmax);
	result += fabs(params1.zrand - params2.zrand);
	result += fabs(params1.sigma_hit - params2.sigma_hit);
	result += fabs(params1.lambda_short - params2.lambda_short);

	return result;
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

	printIntrinsicParams(new_params);

	boost::math::uniform_distribution<> pRand(0.0, 30.0);
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
				double mapDist = distanceAccordingToMap((*scan)->beam_start, beam_end_vec);

				if(mapDist == -1)
					mapDist = 30.0;

				boost::math::normal_distribution<> pHit(mapDist, old_params.sigma_hit);
				boost::math::exponential_distribution<> pShort(old_params.lambda_short);

				tf::Vector3 beam_vec = beam_end_vec - (*scan)->beam_start;
				double beam_length = sqrt(beam_vec.getX()*beam_vec.getX() +
										  beam_vec.getY()*beam_vec.getY() +
										  beam_vec.getZ()*beam_vec.getZ());

				double eta = 1.0/(boost::math::pdf(pHit, beam_length) + boost::math::pdf(pShort, beam_length) + boost::math::pdf(pRand, beam_length) + getPMax(beam_length));

				// ROS_INFO("%3.3f, %3.3f -- %3.3f -- %3.3f, %3.3f, %3.3f, %3.3f", mapDist, beam_length, eta, e_hit, e_short, e_max, e_rand);
				double ehit = eta*boost::math::pdf(pHit, beam_length);
				sigma_update_inner_sum += ehit*(beam_length - mapDist)*(beam_length - mapDist);
				total_ehit += ehit;

				double eshort = eta*boost::math::pdf(pShort, beam_length);
				lambda_update_denom += eshort*beam_length;
				total_eshort += eshort;

				double emax = eta*getPMax(beam_length);
				total_emax += emax;
				double erand = eta*boost::math::pdf(pRand, beam_length);
				total_erand += erand;

				Zcard++;
				
				if(ehit > 1 || eshort > 1 || emax > 1 || erand > 1 || total_ehit/Zcard > 1 || total_eshort/Zcard > 1 || total_emax/Zcard > 1 || total_erand/Zcard > 1)
					ROS_ERROR("%3.3f, %3.3f -- %3.3f -- %3.3f, %3.3f, %3.3f, %3.3f -- %3.3f, %3.3f, %3.3f, %3.3f", mapDist, beam_length, eta, ehit, eshort, emax, erand, total_ehit, total_eshort, total_emax, total_erand);

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
		
		ROS_INFO("Iteration: %d of %d with change of %3.3f", ++iteration, max_iterations, getParameterDelta(new_params, old_params));
		printIntrinsicParams(new_params);
	} while(getParameterDelta(new_params, old_params) > epsilon && iteration < max_iterations);

	ROS_INFO("Test: %3.3f, %3.3f", getPMax(10.0), getPMax(30.0));

	if(iteration < max_iterations)
	{
		ROS_INFO("EM algorithm converged");
	}
	else
	{
		ROS_INFO("EM algorithm failed to converge after %d iterations", max_iterations);
	}


	return new_params;
}

tf::Vector3 rotateVectorByQuat(const tf::Vector3& vec, const tf::Quaternion& quat)
{
	double w1, x1, y1, z1;
	double w2, x2, y2, z2;
	double temp_w, temp_x, temp_y, temp_z;

	// copy to temp variables that makes the math a bit eaiser to read
	w1 = quat.getW();
	x1 = quat.getX();
	y1 = quat.getY();
	z1 = quat.getZ();

	w2 = 0;
	x2 = vec.getX();
	y2 = vec.getY();
	z2 = vec.getZ();

	// calculate q*p
	temp_w = w1*w2 - x1*x2 - y1*y2 - z1*z2;
	temp_x = w1*x2 + x1*w2 + y1*z2 - z1*y2;
	temp_y = w1*y2 - x1*z2 + y1*w2 + z1*x2;
	temp_z = w1*z2 + x1*y2 - y1*x2 + z1*w2;

	// copy to temp variables for the second multiplication
	w1 = temp_w;
	x1 = temp_x;
	y1 = temp_y;
	z1 = temp_z;
	
	tf::Quaternion quatInv = quat.inverse();
	w2 = quatInv.getW();
	x2 = quatInv.getX();
	y2 = quatInv.getY();
	z2 = quatInv.getZ();

	// calculate (q*p)*q^-1
	temp_w = w1*w2 - x1*x2 - y1*y2 - z1*z2;
	temp_x = w1*x2 + x1*w2 + y1*z2 - z1*y2;
	temp_y = w1*y2 - x1*z2 + y1*w2 + z1*x2;
	temp_z = w1*z2 + x1*y2 - y1*x2 + z1*w2;

	tf::Vector3 result(temp_x, temp_y, temp_z);
	return result;
}

// ray traces to find the first collision with an object
// returns the distance along the ray
double distanceAccordingToMap(tf::Vector3 beam_start, tf::Vector3 beam_end)
{
	tf::Vector3 ray_vec = beam_end - beam_start;
	double beam_length = sqrt(ray_vec.getX()*ray_vec.getX() +
							  ray_vec.getY()*ray_vec.getY() +
							  ray_vec.getZ()*ray_vec.getZ());
	tf::Vector3 wall_pos(wall_pose.position.x, wall_pose.position.y, wall_pose.position.z);
	tf::Quaternion wall_rot(wall_pose.orientation.x,
							wall_pose.orientation.y,
							wall_pose.orientation.z,
							wall_pose.orientation.w);

	tf::Vector3 delta = wall_pos - beam_start;

	tf::Vector3 x_axis(1, 0, 0);
	tf::Vector3 wall_x_axis = rotateVectorByQuat(x_axis, wall_rot);

	tf::Vector3 y_axis(0, 1, 0);
	tf::Vector3 wall_y_axis = rotateVectorByQuat(y_axis, wall_rot);

	tf::Vector3 z_axis(0, 0, 1);
	tf::Vector3 wall_z_axis = rotateVectorByQuat(z_axis, wall_rot);

	double tMin = 0;
	double tMax = 1000.0; // max of laser is 30.0 plus noise so this is super overkill

	// calculate intersection with x-planes
	{
		double e = wall_x_axis.dot(delta);
		double f = ray_vec.dot(wall_x_axis);

		if(fabs(f) > .001f)
		{
			double t1 = (e - wall_scale.x/2.0)/f;
			double t2 = (e + wall_scale.x/2.0)/f;
			
			// swap if the wrong order
			if(t1 > t2)
			{
				double temp=t1;
				t1 = t2;
				t2 = temp;
			}
			
			// tMax is the nearest far intersection
			if(t2 < tMax)
				tMax = t2;
			// tMin is the farthest near intersection
			if(t1 > tMin)
				tMin = t1;
			
			if(tMax < tMin)
				return -1;
		}
		else // ray is almsot parallel to the plane
		{
			if(-e - wall_scale.x/2.0 > 0.0f || -e + wall_scale.x/2.0 < 0.0f)
				return -1;
		}
	}

	// calculate intersection with y-planes
	{
		double e = wall_y_axis.dot(delta);
		double f = ray_vec.dot(wall_y_axis);

		if(fabs(f) > .001f)
		{
			double t1 = (e - wall_scale.y/2.0)/f;
			double t2 = (e + wall_scale.y/2.0)/f;
			
			// swap if the wrong order
			if(t1 > t2)
			{
				double temp=t1;
				t1 = t2;
				t2 = temp;
			}
			
			// tMax is the nearest far intersection
			if(t2 < tMax)
				tMax = t2;
			// tMin is the farthest near intersection
			if(t1 > tMin)
				tMin = t1;
			
			if(tMax < tMin)
				return -1;
		}
		else // ray is almsot parallel to the plane
		{
			if(-e - wall_scale.y/2.0 > 0.0f || -e + wall_scale.y/2.0 < 0.0f)
				return -1;
		}
	}

	// calculate intersection with z-planes
	{
		double e = wall_z_axis.dot(delta);
		double f = ray_vec.dot(wall_z_axis);

		if(fabs(f) > .001f)
		{
			double t1 = (e - wall_scale.z/2.0)/f;
			double t2 = (e + wall_scale.z/2.0)/f;
			
			// swap if the wrong order
			if(t1 > t2)
			{
				double temp=t1;
				t1 = t2;
				t2 = temp;
			}
			
			// tMax is the nearest far intersection
			if(t2 < tMax)
				tMax = t2;
			// tMin is the farthest near intersection
			if(t1 > tMin)
				tMin = t1;
			
			if(tMax < tMin)
				return -1;
		}
		else // ray is almsot parallel to the plane
		{
			if(-e - wall_scale.z/2.0 > 0.0f || -e + wall_scale.z/2.0 < 0.0f)
				return -1;
		}
	}

	return tMin*beam_length;
}
