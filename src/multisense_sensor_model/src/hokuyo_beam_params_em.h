#ifndef MULTISENSE_SENSOR_MODEL__HOKUYO_BEAM_PARAMS_EM_H_
#define MULTISENSE_SENSOR_MODEL__HOKUYO_BEAM_PARAMS_EM_H_

#include <multisense_sensor_model/sensor_model.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Pose.h>
#include <pcl_ros/point_cloud.h>
#include <boost/shared_ptr.hpp>
#include <tf/transform_datatypes.h>
#include <pcl_ros/point_cloud.h>
#include <vector>

void printIntrinsicParams(const multisense_sensor_model::IntrinsicParams& params)
{
    ROS_INFO("\nzhit: %3.3f\nzshort: %3.3f\nzmax: %3.3f\nzrand: %3.3f\nsigma_hit: %3.3f\nlambda_short: %3.3f",
			 params.zhit,
			 params.zshort,
			 params.zmax,
			 params.zrand,
			 params.sigma_hit,
			 params.lambda_short);
}

typedef struct ScanData_ {
	pcl::PointCloud<pcl::PointXYZ> beam_ends;
	tf::Vector3 beam_start;
} ScanData;

typedef std::vector<boost::shared_ptr<ScanData> > ScanDataVector;

multisense_sensor_model::IntrinsicParams learnIntrinsicParams(const ScanDataVector laser_scans,
															  const double sigma_hit, const double lambda_short,
															  const double epsilon=1e-5, const int max_iterations=20);

double distanceAccordingToMap(tf::Vector3 beam_start,
							  tf::Vector3 beam_end,
							  geometry_msgs::Pose wall_pose,
							  geometry_msgs::Vector3 wall_scale);

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
double distanceAccordingToMap(tf::Vector3 beam_start,
							  tf::Vector3 beam_end,
							  geometry_msgs::Pose wall_pose,
							  geometry_msgs::Vector3 wall_scale)
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

double getParameterDelta(const multisense_sensor_model::IntrinsicParams& params1,
						 const multisense_sensor_model::IntrinsicParams& params2)
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

#endif
