#include <cylbot_mcl/pose_cloud3d.h>

namespace cylbot_mcl
{
	PoseCloud3D::PoseCloud3D(const RobotModel& model, const int num_samples,
							 const boost::shared_ptr<octomap::OcTree> octree) :
		PoseCloud(model, num_samples)
	{
		this->octree = octree;
	}

	PoseCloud3D::PoseCloud3D(const RobotModel& model,
							 const geometry_msgs::PoseWithCovarianceStamped& initial_pose,
							 const int num_particles,
							 const boost::shared_ptr<octomap::OcTree> octree) :
		PoseCloud(model, initial_pose, num_particles)
	{
		this->octree = octree;
	}

	void PoseCloud3D::sensorUpdate(const geometry_msgs::Pose map_pose,
								   const pcl::PointCloud<pcl::PointXYZ>& beam_ends,
								   const tf::Vector3 beam_start,
								   double curr_time)
	{
		// TODO
	}

	void PoseCloud3D::octreeUpdate(const boost::shared_ptr<octomap::OcTree> octree)
	{
		this->octree = octree;
	}
}
