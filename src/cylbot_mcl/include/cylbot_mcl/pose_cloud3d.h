#ifndef CYLBOT_MCL__POSE_CLOUD3D_H_
#define CYLBOT_MCL__POSE_CLOUD3D_H_

#include <cylbot_mcl/pose_cloud.h>
#include <octomap/OcTree.h>
#include <boost/shared_ptr.hpp>

namespace cylbot_mcl
{
	class PoseCloud3D : public PoseCloud
	{
	public:
		PoseCloud3D(const RobotModel& model, const int num_samples=10000,
					const boost::shared_ptr<octomap::OcTree> octree=boost::shared_ptr<octomap::OcTree>());

		PoseCloud3D(const RobotModel& model,
					const geometry_msgs::PoseWithCovarianceStamped& initial_pose,
					const int num_particles=1000,
					const boost::shared_ptr<octomap::OcTree> octree=boost::shared_ptr<octomap::OcTree>());

		void sensorUpdate(const geometry_msgs::Pose map_pose,
						  const pcl::PointCloud<pcl::PointXYZ>& beam_ends,
						  const tf::Vector3 beam_start,
						  double curr_time);

		void octreeUpdate(const boost::shared_ptr<octomap::OcTree> octree);

	private:
		boost::shared_ptr<octomap::OcTree> octree;

		double last_sensor_update;
	};
}

#endif
