#ifndef CYLBOT_MCL__POSE_CLOUD_H_
#define CYLBOT_MCL__POSE_CLOUD_H_

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Twist.h>
#include <pcl_ros/point_cloud.h>
#include <tf/transform_datatypes.h>
#include <cylbot_map_creator/LikelihoodField.h>
#include <cylbot_motion_model/multivariate_normal.h>
#include <boost/random.hpp>
#include <multisense_sensor_model/sensor_model.h>
#include <boost/array.hpp>
#include <utility>

namespace cylbot_mcl
{
	typedef struct RobotModel_
	{
		RobotModel_()
			{
				for(int i=0; i<alpha.size(); i++)
				{
					alpha[i] = .1;
				}
			}

		RobotModel_(multisense_sensor_model::IntrinsicParams params)
			{
				sensor_params = params;
				for(int i=0; i<alpha.size(); i++)
				{
					alpha[i] = .1;
				}
			}

		multisense_sensor_model::IntrinsicParams sensor_params;
		boost::array<double, 6> alpha;
	} RobotModel;

	
	class PoseCloud2D
	{
	public:
		PoseCloud2D(const RobotModel& model, 
					const cylbot_map_creator::LikelihoodField& field=cylbot_map_creator::LikelihoodField());
		PoseCloud2D(const RobotModel& model,
					const geometry_msgs::PoseWithCovarianceStamped& initial_pose,
					const int num_particles=1000,
					const cylbot_map_creator::LikelihoodField& field=cylbot_map_creator::LikelihoodField());
		
		void resetCloud(const geometry_msgs::PoseWithCovarianceStamped& initial_pose, const int num_particles=1000);

		void motionUpdate(const geometry_msgs::Twist& u, double dt);

		void sensorUpdate(const pcl::PointCloud<pcl::PointXYZ>& beam_ends, double curr_time);

		void fieldUpdate(const cylbot_map_creator::LikelihoodField& field);

		geometry_msgs::PoseArray getPoses();

	private:
		int getCellDistance(const int x, const int y);
		geometry_msgs::PoseArray generatePoses(const geometry_msgs::PoseWithCovarianceStamped& initial_pose, const int num_particles);
		geometry_msgs::PoseArray generateUniformPoses(const std::pair<double, double> x_range,
													  const std::pair<double, double> y_range,
													  const int num_poses);

	public:
		double getMeasurementProbability(const geometry_msgs::Pose& pose,
										 const pcl::PointCloud<pcl::PointXYZ>& beam_ends);

	private:
		RobotModel model;
		cylbot_map_creator::LikelihoodField likelihood_field;
		geometry_msgs::PoseArray pose_array;
		Eigen::internal::scalar_normal_dist_op<double> randn;
		boost::mt19937 rng;
		geometry_msgs::Twist last_cmd;

		double last_sensor_update;
		double w_slow, w_fast;
		double alpha_slow, alpha_fast;

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
