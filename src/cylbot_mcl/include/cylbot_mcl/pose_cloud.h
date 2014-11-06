#ifndef CYLBOT_MCL__POSE_CLOUD_H_
#define CYLBOT_MCL__POSE_CLOUD_H_

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Twist.h>
#include <pcl_ros/point_cloud.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/OccupancyGrid.h>
#include <cylbot_motion_model/multivariate_normal.h>
#include <boost/random.hpp>
#include <multisense_sensor_model/sensor_model.h>
#include <boost/array.hpp>

namespace cylbot_mcl
{
	typedef struct RobotModel_
	{
		multisense_sensor_model::IntrinsicParams sensor_params;
		boost::array<double, 6> alpha;
	} RobotModel;

	
	class PoseCloud2D
	{
	public:
		PoseCloud2D(const RobotModel& model, const nav_msgs::OccupancyGrid& map=nav_msgs::OccupancyGrid());
		PoseCloud2D(const RobotModel& model,
					const geometry_msgs::PoseWithCovarianceStamped& initial_pose,
					const int num_particles=1000,
					const nav_msgs::OccupancyGrid& map=nav_msgs::OccupancyGrid());
		
		void resetCloud(const geometry_msgs::PoseWithCovarianceStamped& initial_pose, const int num_particles=1000);

		void motionUpdate(const geometry_msgs::Twist& u, double dt);

		void sensorUpdate(const pcl::PointCloud<pcl::PointXYZ>& beam_ends,
						  const tf::Vector3& beam_start);

		void mapUpdate(const nav_msgs::OccupancyGrid& map);

		geometry_msgs::PoseArray getPoses();

	private:
		RobotModel model;
		nav_msgs::OccupancyGrid map;
		geometry_msgs::PoseArray pose_array;
		Eigen::internal::scalar_normal_dist_op<double> randn;
		boost::mt19937 rng;
		geometry_msgs::Twist last_cmd;

	};

	template <class T1, class T2>
		bool sort_pair_second(const std::pair<T1,T2>&left, const std::pair<T1,T2>&right)
	{
		return left.second < right.second;
	}

	template<class T1, class T2, class T3>
		bool lower_bound_pair_second(const std::pair<T1, T2>& left, const T3& right)
	{
		return left.second < right;
	}
}

#endif
