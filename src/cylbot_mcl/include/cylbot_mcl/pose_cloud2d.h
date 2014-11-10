#ifndef CYLBOT_MCL__POSE_CLOUD2D_H_
#define CYLBOT_MCL__POSE_CLOUD2D_H_

#include <cylbot_mcl/pose_cloud.h>

namespace cylbot_mcl
{
	class PoseCloud2D : public PoseCloud
	{
	public:
		PoseCloud2D(const RobotModel& model, const int num_samples=10000,
					const cylbot_map_creator::LikelihoodField& field=cylbot_map_creator::LikelihoodField());
		PoseCloud2D(const RobotModel& model,
					const geometry_msgs::PoseWithCovarianceStamped& initial_pose,
					const int num_particles=1000,
					const cylbot_map_creator::LikelihoodField& field=cylbot_map_creator::LikelihoodField());
		
		void sensorUpdate(const geometry_msgs::Pose map_pose,
						  const pcl::PointCloud<pcl::PointXYZ>& beam_ends,
						  double curr_time);

		void fieldUpdate(const cylbot_map_creator::LikelihoodField& field);

	private:
		int getCellDistance(const int x, const int y);

	public:
		double getMeasurementProbability(const geometry_msgs::Pose& map_pose,
										 const geometry_msgs::Pose& pose,
										 const pcl::PointCloud<pcl::PointXYZ>& beam_ends);

	private:
		cylbot_map_creator::LikelihoodField likelihood_field;

		double last_sensor_update;
		double w_slow, w_fast;
		double alpha_slow, alpha_fast;

	};
}

#endif
