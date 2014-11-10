#include <cylbot_mcl/pose_cloud2d.h>
#include <boost/random/normal_distribution.hpp>
#include <boost/random/uniform_real_distribution.hpp>
#include <boost/bind.hpp>
#include <cmath>
#include <vector>
#include <utility>
#include <algorithm>
#include <tf/transform_datatypes.h>
#include <limits>
#include <set>

namespace cylbot_mcl
{
	PoseCloud2D::PoseCloud2D(const RobotModel& model, const int num_samples, const cylbot_map_creator::LikelihoodField& field) :
		PoseCloud(model, num_samples)
	{
		this->likelihood_field = field;

		last_sensor_update = 0;
		w_slow = w_fast = 0;
	}

	PoseCloud2D::PoseCloud2D(const RobotModel& model,
							 const geometry_msgs::PoseWithCovarianceStamped& initial_pose,
							 const int num_particles,
							 const cylbot_map_creator::LikelihoodField& field) :
		PoseCloud(model, initial_pose, num_particles)
	{
		this->likelihood_field = field;
		
		last_sensor_update = 0;
		w_slow = w_fast = 0;
	}

	void PoseCloud2D::sensorUpdate(const geometry_msgs::Pose map_pose,
								   const pcl::PointCloud<pcl::PointXYZ>& beam_ends,
								   double curr_time)
	{
		// this type is used as a temporary datastructure in this function only
		// this typedef just saves some typing and makes the later code more readable
		typedef geometry_msgs::PoseArray::_poses_type::iterator WeightPairFirst;
		typedef double WeightPairSecond;
		typedef std::pair<WeightPairFirst, WeightPairSecond> WeightPair;
		typedef std::vector<WeightPair> Weights;
		
		// if we haven't set a map yet then don't do anything
		if(likelihood_field.data.size() == 0)
			return;
		
		// don't perform sensor updates unless moving
		// this is to prevent the variance from trending to 0 with probability 1
		// see page 108 Resampling section of Probabilistic Robotics Textbook
		if(fabs(last_cmd.linear.y) < 0.1 && fabs(last_cmd.angular.z) < 0.1)
			return;

		// update at most every 5 seconds
		if(curr_time - last_sensor_update < 1)
			return;

		last_sensor_update = curr_time;

		// store an iterator to a pose along with its computed weight
		Weights pose_weights;
		double total_weight = 0;
		double max_weight = 0;

		//ROS_INFO("Starting calculation of true pose probability");
		double true_prob = getMeasurementProbability(map_pose, map_pose, beam_ends);
		ROS_INFO_STREAM("Finished calculation of true pose probability " << true_prob);

		// for(pcl::PointCloud<pcl::PointXYZ>::const_iterator beam_end = beam_ends.points.begin();
		// 	beam_end != beam_ends.points.end();
		// 	beam_end++)
		// {
		// 	if(beam_end->x != beam_end->x || beam_end->y != beam_end->y || beam_end->z != beam_end->z)
		// 		ROS_WARN("sensorUpdate: beam_end has NaN value");
		// }

		// calculate the weight of each pose particle
		//ROS_INFO("Starting calculation of pose cloud probabilities");
		for(geometry_msgs::PoseArray::_poses_type::iterator pose = pose_array.poses.begin();
			pose != pose_array.poses.end();
			pose++)
		{
			double prob = getMeasurementProbability(map_pose, *pose, beam_ends);
			if(prob < 0)
				continue; // ignore points which had no beam
			pose_weights.push_back(std::make_pair(pose, prob));
			total_weight += prob;

			if(prob > max_weight)
				max_weight = prob;
		}

		// if there was no valid sensor data
		// then we cannot perform an update
		if(pose_weights.size() == 0)
			return;
		
		//ROS_INFO_STREAM("Finished calculation of pose cloud probabilities");
		double avg_weight = total_weight/pose_array.poses.size();

		w_slow = w_slow + model.alpha_slow*(avg_weight - w_slow);
		w_fast = w_fast + model.alpha_fast*(avg_weight - w_fast);
		ROS_INFO_STREAM("total weight: " << total_weight);
		ROS_INFO_STREAM("Average prob: " << total_weight/pose_array.poses.size());
		ROS_INFO_STREAM("Max prob: " << max_weight);

		// normalize
		for(Weights::iterator weight_pair = pose_weights.begin();
			weight_pair != pose_weights.end();
			weight_pair++)
		{
			weight_pair->second /= total_weight;
			//ROS_INFO_STREAM(weight_pair->second);
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

	void PoseCloud2D::fieldUpdate(const cylbot_map_creator::LikelihoodField& field)
	{
		this->likelihood_field = field;
	}

	double PoseCloud2D::getMeasurementProbability(const geometry_msgs::Pose& map_pose,
												  const geometry_msgs::Pose& pose,
												  const pcl::PointCloud<pcl::PointXYZ>& beam_ends)
	{
		double probability = -1.0;

		int x_origin = (int)fabs(likelihood_field.info.origin.position.x/likelihood_field.info.resolution);
		int y_origin = (int)fabs(likelihood_field.info.origin.position.y/likelihood_field.info.resolution);

		// construct transfrom from map to estimated pose
		tf::Transform pose_transform;
		pose_transform.setOrigin(tf::Vector3(pose.position.x, pose.position.y, pose.position.z));
		// the pose estimates align along the y-axis, but the tf package uses the x-axis as a reference
		// so need to rotate the pose estimates by -90 degrees back to the x-axis
		tf::Quaternion pose_rotation = tf::createQuaternionFromRPY(0, 0, tf::getYaw(map_pose.orientation) - 3.14159/2.0);
		pose_transform.setRotation(pose_rotation);

		//ROS_INFO("Starting scan probability calculation");
		for(pcl::PointCloud<pcl::PointXYZ>::const_iterator beam_end = beam_ends.points.begin();
			beam_end != beam_ends.points.end();
			beam_end++)
		{
			tf::Vector3 map_beam_end = pose_transform(tf::Vector3(beam_end->x, beam_end->y, beam_end->z));
			
			// convert to grid coordinates
			int x = round(map_beam_end.getX()/likelihood_field.info.resolution);
			int y = round(map_beam_end.getY()/likelihood_field.info.resolution);
			double distance = getCellDistance(x + x_origin, y + y_origin);

			//ROS_INFO_STREAM("distance:" << distance);

			// lookup failed so skip this beam
			if(distance == -1)
			{
				//ROS_INFO("Failed to lookup point (%d, %d) -- (%d, %d)", x, y, x + x_origin, y + y_origin);

				// assume every point not in the map has an equal probability of hit
				// in this case inversely proportional to the probability of hitting a maximum
				if(probability < 0)
					probability = model.sensor_params.zmax;
				else
					probability *= model.sensor_params.zmax;

				continue;
			}
			
			double beam_prob = model.sensor_params.likelihoodProbability(distance);
			if(probability < 0)
				probability = beam_prob;
			else
				probability *= beam_prob;
			
		}

		//ROS_INFO_STREAM("Finished calculating probability of " << probability);
		return probability;
	}

	int PoseCloud2D::getCellDistance(const int x, const int y)
	{
		if(x < likelihood_field.info.width && x >= 0 &&
		   y < likelihood_field.info.height && y >= 0)
		{
			return likelihood_field.data[y*likelihood_field.info.width + x];
		}

		return -1;
	}
}
