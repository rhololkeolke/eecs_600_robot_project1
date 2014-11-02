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

IntrinsicParams learnIntrinsicParams(const ScanDataVector laser_scans, double sigma_hit, double lambda_short);

double distanceAccordingToMap(tf::Vector3 beam_start, tf::Vector3 beam_end);

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

	IntrinsicParams params = learnIntrinsicParams(laser_scans, .01, 1.0);
}

IntrinsicParams learnIntrinsicParams(const ScanDataVector laser_scans,
									 double sigma_hit, double lambda_short)
{
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

			tf::Vector3 beam_vec = beam_end_vec - (*scan)->beam_start;
			double beam_length = sqrt(beam_vec.getX()*beam_vec.getX() +
									  beam_vec.getY()*beam_vec.getY() +
									  beam_vec.getZ()*beam_vec.getZ());

			ROS_INFO("%3.3f --- %3.3f", mapDist, beam_length);
		}
	}
	
	IntrinsicParams params;
	return params;
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
