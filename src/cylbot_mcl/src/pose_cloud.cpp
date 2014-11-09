#include <cylbot_mcl/pose_cloud.h>
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
	inline int round(const double a) { return int (a + 0.5); }
	
	PoseCloud2D::PoseCloud2D(const RobotModel& model, const cylbot_map_creator::LikelihoodField& field)
	{
		this->pose_array.header.frame_id = "/map";

		this->model = model;
		
		this->likelihood_field = field;

		last_sensor_update = 0;

		w_slow = w_fast = 0;
		alpha_slow = .1;
		alpha_fast = 1;

		pose_array.poses = generateUniformPoses(std::make_pair(-20.0, 20.0),
												std::make_pair(-20.0, 20.0), 10000).poses;
	}

	PoseCloud2D::PoseCloud2D(const RobotModel& model,
							 const geometry_msgs::PoseWithCovarianceStamped& initial_pose,
							 const int num_particles,
							 const cylbot_map_creator::LikelihoodField& field)
	{
		this->pose_array.header.frame_id = "/map";

		this->model = model;

		this->likelihood_field = field;
		
		this->resetCloud(initial_pose, num_particles);

		last_sensor_update = 0;

		w_slow = w_fast = 0;
		alpha_slow = .1;
		alpha_fast = 1;

		pose_array.poses = generateUniformPoses(std::make_pair(-20.0, 20.0),
												std::make_pair(-20.0, 20.0), 10000).poses;
	}

	void PoseCloud2D::resetCloud(const geometry_msgs::PoseWithCovarianceStamped& initial_pose, const int num_particles)
	{
		pose_array.poses = generatePoses(initial_pose, num_particles).poses;
	}

	geometry_msgs::PoseArray PoseCloud2D::generatePoses(const geometry_msgs::PoseWithCovarianceStamped& initial_pose,
														const int num_particles)
	{
		Eigen::VectorXd mean(3);
		Eigen::MatrixXd covar(3,3);

		mean << initial_pose.pose.pose.position.x,
			    initial_pose.pose.pose.position.y,
			    tf::getYaw(initial_pose.pose.pose.orientation);
		covar << initial_pose.pose.covariance[0], 0,                                  0,
			     0,                               initial_pose.pose.covariance[7],    0,
			     0,                               0,                                  initial_pose.pose.covariance[35];

		Eigen::MatrixXd normTransform(3, 3);
		Eigen::LLT<Eigen::MatrixXd> cholSolver(covar);
		
		if(cholSolver.info() == Eigen::Success)
		{
			normTransform = cholSolver.matrixL();
		}
		else
		{
			Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigenSolver(covar);
			normTransform = eigenSolver.eigenvectors() * eigenSolver.eigenvalues().cwiseSqrt().asDiagonal();
		}
		
		Eigen::MatrixXd samples = (normTransform * Eigen::MatrixXd::NullaryExpr(3, num_particles, randn)).colwise() + mean;

		geometry_msgs::PoseArray new_pose_array;
		for(int i=0; i<num_particles; i++)
		{
			geometry_msgs::Pose pose;
			pose.position.x = samples(0, i);
			pose.position.y = samples(1, i);
			pose.position.z = 0;

			// generate random yaw then convert it to a quaternion
			pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, samples(2, i));
		
			new_pose_array.poses.push_back(pose);
		}

		
		return new_pose_array;
	}

	geometry_msgs::PoseArray PoseCloud2D::generateUniformPoses(const std::pair<double, double> x_range,
															   const std::pair<double, double> y_range,
															   const int num_poses)
	{
		boost::random::uniform_real_distribution<> x_dist(x_range.first, x_range.second);
		boost::random::uniform_real_distribution<> y_dist(y_range.first, y_range.second);
		boost::random::uniform_real_distribution<> yaw_dist(0.0, 2*3.14159);

		boost::variate_generator<boost::mt19937, boost::random::uniform_real_distribution<> > x_rand(rng, x_dist);
		boost::variate_generator<boost::mt19937, boost::random::uniform_real_distribution<> > yaw_rand(rng, yaw_dist);
		
		geometry_msgs::PoseArray new_pose_array;
		for(int i=0; i<num_poses; i++)
		{
			geometry_msgs::Pose pose;
			pose.position.x = x_rand();
			pose.position.y = x_rand();
			pose.position.z = 0;

			pose.orientation = tf::createQuaternionMsgFromYaw(yaw_rand());

			new_pose_array.poses.push_back(pose);
		}

		return new_pose_array;
	}

	void PoseCloud2D::motionUpdate(const geometry_msgs::Twist& u, double dt)
	{
		last_cmd = u;
		// don't do anything if the robot isn't moving
		if(fabs(u.linear.y) < 0.1 && fabs(u.angular.z) < 0.1)
			return;

		double v_hat_variance = model.alpha[0]*u.linear.y*u.linear.y + model.alpha[1]*u.angular.z*u.angular.z;
		double w_hat_variance = model.alpha[2]*u.linear.y*u.linear.y + model.alpha[3]*u.angular.z*u.angular.z;
		double gamma_variance = model.alpha[4]*u.linear.y*u.linear.y + model.alpha[5]*u.angular.z*u.angular.z;
		
		boost::normal_distribution<> v_hat_dist(0.0, sqrt(v_hat_variance));
		boost::normal_distribution<> w_hat_dist(0.0, sqrt(w_hat_variance));
		boost::normal_distribution<> gamma_dist(0.0, sqrt(gamma_variance));
		
		double v_hat = u.linear.y + v_hat_dist(rng);
		double w_hat = u.angular.z + w_hat_dist(rng);
		double gamma = gamma_dist(rng);
		
		double v_w_ratio = v_hat/w_hat;

		for(geometry_msgs::PoseArray::_poses_type::iterator pose = pose_array.poses.begin();
			pose != pose_array.poses.end();
			pose++)
		{
			double yaw = tf::getYaw(pose->orientation);

			double x_prime = pose->position.x - v_w_ratio*sin(yaw) + v_w_ratio*sin(yaw + w_hat*dt);
			double y_prime = pose->position.y + v_w_ratio*cos(yaw) - v_w_ratio*cos(yaw + w_hat*dt);
			double yaw_prime = yaw + w_hat*dt + gamma*dt;

			pose->position.x = x_prime;
			pose->position.y = y_prime;
			pose->orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, yaw_prime);
		}

	}

	void PoseCloud2D::sensorUpdate(const pcl::PointCloud<pcl::PointXYZ>& beam_ends, double curr_time)
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
		if(fabs(last_cmd.linear.y) < 0.1 && fabs(last_cmd.linear.z) < 0.1)
			return;

		// update at most every 5 seconds
		if(curr_time - last_sensor_update < 5)
			return;

		last_sensor_update = curr_time;

		// store an iterator to a pose along with its computed weight
		Weights pose_weights;
		double total_weight = 0;
		double max_weight = 0;

		// for(pcl::PointCloud<pcl::PointXYZ>::const_iterator beam_end = beam_ends.points.begin();
		// 	beam_end != beam_ends.points.end();
		// 	beam_end++)
		// {
		// 	if(beam_end->x != beam_end->x || beam_end->y != beam_end->y || beam_end->z != beam_end->z)
		// 		ROS_WARN("sensorUpdate: beam_end has NaN value");
		// }

		// calculate the weight of each pose particle
		for(geometry_msgs::PoseArray::_poses_type::iterator pose = pose_array.poses.begin();
			pose != pose_array.poses.end();
			pose++)
		{
			double prob = getMeasurementProbability(*pose, beam_ends);
			pose_weights.push_back(std::make_pair(pose, prob));
			total_weight += prob;

			if(prob > max_weight)
				max_weight = prob;
		}
		double avg_weight = total_weight/pose_array.poses.size();

		w_slow = w_slow + alpha_slow*(avg_weight - w_slow);
		w_fast = w_fast + alpha_fast*(avg_weight - w_fast);
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
			if(new_sample_rand() < 1.0 - w_fast/w_slow)
			{
				poses.push_back(generateUniformPoses(std::make_pair(-20.0, 20.0),
													 std::make_pair(-20.0, 20.0),
													 1).poses[0]);
			}
			else
			{
				pose_set.insert(&(*(pose_weights[i].first)));
			}
		}

		for(std::set<geometry_msgs::Pose*>::const_iterator pose = pose_set.begin();
			pose != pose_set.end();
			pose++)
		{
			poses.push_back(*(*pose));
		}

		pose_array.poses = poses;
	}

	void PoseCloud2D::fieldUpdate(const cylbot_map_creator::LikelihoodField& field)
	{
		this->likelihood_field = field;
	}

	geometry_msgs::PoseArray PoseCloud2D::getPoses()
	{
		return this->pose_array;
	}

	double PoseCloud2D::getMeasurementProbability(const geometry_msgs::Pose& pose,
												  const pcl::PointCloud<pcl::PointXYZ>& beam_ends)
	{
		double probability = 1.0;

		int x_origin = (int)fabs(likelihood_field.info.origin.position.x/likelihood_field.info.resolution);
		int y_origin = (int)fabs(likelihood_field.info.origin.position.y/likelihood_field.info.resolution);

		// construct transfrom from map to estimated pose
		tf::Transform pose_transform;
		pose_transform.setOrigin(tf::Vector3(pose.position.x, pose.position.y, pose.position.z));
		tf::Quaternion pose_rotation;
		tf::quaternionMsgToTF(pose.orientation, pose_rotation);
		pose_transform.setRotation(pose_rotation);

		for(pcl::PointCloud<pcl::PointXYZ>::const_iterator beam_end = beam_ends.points.begin();
			beam_end != beam_ends.points.end();
			beam_end++)
		{

			tf::Vector3 map_beam_end = pose_transform(tf::Vector3(beam_end->x, beam_end->y, beam_end->z));
			
			// convert to grid coordinates
			int x = map_beam_end.getX()/likelihood_field.info.resolution;
			int y = map_beam_end.getY()/likelihood_field.info.resolution;
			double distance = getCellDistance(round(x) + x_origin, round(y) + y_origin);

			// lookup failed so skip this beam
			if(distance == -1)
				continue;
			
			double beam_prob = model.sensor_params.likelihoodProbability(distance)/100.0;
			probability = probability*beam_prob;
		}
		
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
