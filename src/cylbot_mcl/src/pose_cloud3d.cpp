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
		// this type is used as a temporary datastructure in this function only
		// this typedef just saves some typing and makes the later code more readable
		typedef geometry_msgs::PoseArray::_poses_type::iterator WeightPairFirst;
		typedef double WeightPairSecond;
		typedef std::pair<WeightPairFirst, WeightPairSecond> WeightPair;
		typedef std::vector<WeightPair> Weights;

		// if there is no map then can't do anything
		if(octree == NULL || octree->size() == 0)
			return;

		// don't perform sensor updates unless moving
		// this is to prevent the variance from trending to 0 with probability 1
		// see page 108 Resampling section of Probabilistic Robotics Textbook
		if(fabs(last_cmd.linear.y) < 0.1 && fabs(last_cmd.angular.z) < 0.1)
			return;

		// update at most every second
		if(curr_time - last_sensor_update < 1 )
			return;

		last_sensor_update = curr_time;

		Weights pose_weights;
		double total_weight = 0;
		double max_weight = 0;

		for(geometry_msgs::PoseArray::_poses_type::iterator pose = pose_array.poses.begin();
			pose != pose_array.poses.end();
			pose++)
		{
			double prob = getMeasurementProbability(map_pose, *pose, beam_ends, beam_start);
			if(prob < 0)
				continue; // ignore points which had no beam
			pose_weights.push_back(std::make_pair(pose, prob));
			total_weight += prob;

			if(prob > max_weight)
				max_weight = prob;
		}

		// if there was no valid sensor data
		// then no update can be performed
		if(pose_weights.size() == 0)
			return;

		double avg_weight = total_weight/pose_array.poses.size();

		ROS_INFO_STREAM("total weight: " << total_weight);
		ROS_INFO_STREAM("average weight: " << avg_weight);
		ROS_INFO_STREAM("max weight: " << max_weight);

		// normalize
		for(Weights::iterator weight_pair = pose_weights.begin();
			weight_pair != pose_weights.end();
			weight_pair++)
		{
			weight_pair->second /= total_weight;
		}

		double M = (double)(pose_weights.size());

		// create prob generator
		boost::random::uniform_real_distribution<> dist(0.0, 1.0/M);
		boost::random::uniform_real_distribution<> new_sample_dist(0.0, 1.0);
		boost::variate_generator<boost::mt19937, boost::random::uniform_real_distribution<> > rand(rng, dist);
		boost::variate_generator<boost::mt19937, boost::random::uniform_real_distribution<> > new_sample_rand(rng, new_sample_dist);

		// resample using low variance sampling from table 4.4 on page 110 of the Probabilistic robotics book
		std::set<geometry_msgs::Pose*> pose_set;
		geometry_msgs::PoseArray::_poses_type poses;
		
		double r = rand();
		double c = pose_weights.begin()->second;
		int i=0;
		for(int m=0; m<M; m++)
		{
			double U = r + m/M;
			while(U > c)
			{
				i++;
				c = c + pose_weights[i].second;
			}

			// if(new_sample_rand() < 1.0 - w_fast/w_slow)
			// {
			// 	// geometry_msgs::PoseArray randomPoses = generateUniformPoses(std::make_pair(-20.0, 20.0),
			// 	// 															std::make_pair(-20.0, 20.0),
			// 	// 															100);
			// 	// for(int i=0; i<randomPoses.poses.size(); i++)
			// 	// 	poses.push_back(randomPoses.poses[i]);

			// 	geometry_msgs::PoseWithCovarianceStamped map_pose_with_cov;
			// 	map_pose_with_cov.pose.pose = map_pose;
			// 	map_pose_with_cov.pose.covariance[0] = map_pose_with_cov.pose.covariance[7] = map_pose_with_cov.pose.covariance[35] = .001;

			// 	geometry_msgs::PoseArray new_samples = generatePoses(map_pose_with_cov, 1);
			// 	for(geometry_msgs::PoseArray::_poses_type::const_iterator pose = new_samples.poses.begin();
			// 		pose != new_samples.poses.end();
			// 		pose++)
			// 	{
			// 		poses.push_back(*pose);
			// 	}

			// }
			// else
			{
				pose_set.insert(&(*(pose_weights[i].first)));
				// poses.push_back(*(pose_weights[i].first));
			}
		}

		for(std::set<geometry_msgs::Pose*>::const_iterator pose = pose_set.begin();
			pose != pose_set.end();
			pose++)
		{
			poses.push_back(*(*pose));
		}

		// repeat some of the points if there are only a few
		i=0;
		while(poses.size() < 100)
		{
			poses.push_back(poses[i++%poses.size()]);
		}

		pose_array.poses = poses;		
	}

	void PoseCloud3D::octreeUpdate(const boost::shared_ptr<octomap::OcTree> octree)
	{
		this->octree = octree;
	}

	double PoseCloud3D::getMeasurementProbability(const geometry_msgs::Pose map_pose,
												  const geometry_msgs::Pose pose,
												  const pcl::PointCloud<pcl::PointXYZ>& beam_ends,
												  const tf::Vector3 beam_start)
	{
		return 1.0;
	}
}
