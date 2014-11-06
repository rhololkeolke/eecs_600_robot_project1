#include <cylbot_mcl/pose_cloud.h>
#include <boost/random/normal_distribution.hpp>
#include <boost/random/uniform_real_distribution.hpp>
#include <boost/bind.hpp>
#include <cmath>
#include <vector>
#include <utility>
#include <algorithm>

namespace cylbot_mcl
{
	PoseCloud2D::PoseCloud2D(const RobotModel& model, const nav_msgs::OccupancyGrid& map)
	{
		this->pose_array.header.frame_id = "/map";

		this->model = model;
		
		this->map = map;
	}

	PoseCloud2D::PoseCloud2D(const RobotModel& model,
							 const geometry_msgs::PoseWithCovarianceStamped& initial_pose,
							 const int num_particles,
							 const nav_msgs::OccupancyGrid& map)
	{
		this->pose_array.header.frame_id = "/map";

		this->model = model;

		this->map = map;
		
		this->resetCloud(initial_pose, num_particles);
	}

	void PoseCloud2D::resetCloud(const geometry_msgs::PoseWithCovarianceStamped& initial_pose, const int num_particles)
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
		
		pose_array.poses.clear();
		for(int i=0; i<num_particles; i++)
		{
			geometry_msgs::Pose pose;
			pose.position.x = samples(0, i);
			pose.position.y = samples(1, i);
			pose.position.z = 0;

			// generate random yaw then convert it to a quaternion
			pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, samples(2, i));
		
			pose_array.poses.push_back(pose);
		}
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

	void PoseCloud2D::sensorUpdate(const pcl::PointCloud<pcl::PointXYZ>& beam_ends,
								   const tf::Vector3& beam_start)
	{
		// this type is used as a temporary datastructure in this function only
		// this typedef just saves some typing and makes the later code more readable
		typedef geometry_msgs::PoseArray::_poses_type::iterator WeightPairFirst;
		typedef double WeightPairSecond;
		typedef std::pair<WeightPairFirst, WeightPairSecond> WeightPair;
		typedef std::vector<WeightPair> Weights;
		
		// if we haven't set a map yet then don't do anything
		if(map.data.size() == 0)
			return;
		
		// don't perform sensor updates unless moving
		// this is to prevent the variance from trending to 0 with probability 1
		// see page 108 Resampling section of Probabilistic Robotics Textbook
		if(fabs(last_cmd.linear.y) < 0.1 && fabs(last_cmd.linear.z) < 0.1)
			return;

		// store an iterator to a pose along with its computed weight
		Weights pose_weights;
		double total_weight = 0;

		// calculate the weight of each pose particle
		for(geometry_msgs::PoseArray::_poses_type::iterator pose = pose_array.poses.begin();
			pose != pose_array.poses.end();
			pose++)
		{
			// TODO: for now just push back a weight of 1 for each point
			pose_weights.push_back(std::make_pair(pose, 1.0));
			total_weight += 1.0;
		}

		// normalize the weight of each pose particle
		for(Weights::iterator pose_weight = pose_weights.begin();
			pose_weight != pose_weights.end();
			pose_weight++)
		{
			pose_weight->second /= total_weight;
		}

		// sort the vector by the weights (i.e. the second element)
		// this is the first step to converting the weights to a CDF for sampling
		std::sort(pose_weights.begin(), pose_weights.end(), sort_pair_second<WeightPairFirst, WeightPairSecond>);

		// convert to CDF
		for(int i=1; i<pose_weights.size(); i++)
		{
			pose_weights[i].second += pose_weights[i-1].second;
		}

		// create prob generator
		boost::random::uniform_real_distribution<> dist;
		boost::variate_generator<boost::mt19937, boost::random::uniform_real_distribution<> > rand(rng, dist);

		// resample
		geometry_msgs::PoseArray::_poses_type poses;
		for(int i=0; i<pose_weights.size(); i++)
		{
			//Weights::iterator weight_pair;
			Weights::iterator weight_pair = std::lower_bound<Weights::iterator, double>(pose_weights.begin(),
																						pose_weights.end(),
																						rand(),
																						lower_bound_pair_second<WeightPairFirst,
																						WeightPairSecond, double>);
			poses.push_back(*(weight_pair->first));
		}

		// update the main datastructure array
		pose_array.poses = poses;
	}

	void PoseCloud2D::mapUpdate(const nav_msgs::OccupancyGrid& map)
	{
		this->map = map;
	}

	geometry_msgs::PoseArray PoseCloud2D::getPoses()
	{
		return this->pose_array;
	}
	
}
