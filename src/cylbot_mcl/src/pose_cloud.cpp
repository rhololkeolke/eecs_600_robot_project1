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
	PoseCloud::PoseCloud(const RobotModel& model, const int num_samples) :
		model(model)
	{
		this->pose_array.header.frame_id = "/map";

		this->pose_array.poses = generateUniformPoses(std::make_pair(-20.0, 20.0),
													  std::make_pair(-20.0, 20.0),
													  num_samples).poses;
	}

	PoseCloud::PoseCloud(const RobotModel& model,
						 const geometry_msgs::PoseWithCovarianceStamped& initial_pose,
						 const int num_particles) :
		model(model)
	{
		this->pose_array.header.frame_id = "/map";
		this->resetCloud(initial_pose, num_particles);
	}

	void PoseCloud::resetCloud(const geometry_msgs::PoseWithCovarianceStamped& initial_pose, const int num_particles)
	{
		pose_array.poses = generatePoses(initial_pose, num_particles).poses;
	}

	geometry_msgs::PoseArray PoseCloud::generatePoses(const geometry_msgs::PoseWithCovarianceStamped& initial_pose,
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

	geometry_msgs::PoseArray PoseCloud::generateUniformPoses(const std::pair<double, double> x_range,
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

	void PoseCloud::motionUpdate(const geometry_msgs::Twist& u, double dt)
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

	geometry_msgs::PoseArray PoseCloud::getPoses()
	{
		return this->pose_array;
	}
	
}
