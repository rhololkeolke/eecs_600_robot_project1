#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <tf/transform_datatypes.h>
#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>
#include <cmath>
#include <cylbot_motion_model/multivariate_normal.h>
#include <iostream>

#define NUM_POSES 1000

geometry_msgs::PoseArray pose_array;

Eigen::internal::scalar_normal_dist_op<double> randN;
//Eigen::internal::scalar_normal_dist_op<double>::rng.seed(1);

double alpha[6];

// random number generator
boost::mt19937 rng;

geometry_msgs::TwistStamped::ConstPtr curr_vel_cmd;

void sampleVelocityMotionModel(const geometry_msgs::Twist& u, \
							   geometry_msgs::PoseArray::_poses_type::iterator x,
							   double dt,
							   const double (&alpha)[6],
							   boost::mt19937& rng)
{
	double v_hat_variance = alpha[0]*u.linear.y*u.linear.y + alpha[1]*u.angular.z*u.angular.z;
	double w_hat_variance = alpha[2]*u.linear.y*u.linear.y + alpha[3]*u.angular.z*u.angular.z;
	double gamma_variance = alpha[4]*u.linear.y*u.linear.y + alpha[5]*u.angular.z*u.angular.z;

	boost::normal_distribution<> v_hat_dist(0.0, sqrt(v_hat_variance));
	boost::normal_distribution<> w_hat_dist(0.0, sqrt(w_hat_variance));
	boost::normal_distribution<> gamma_dist(0.0, sqrt(gamma_variance));

	double v_hat = u.linear.y + v_hat_dist(rng);
	double w_hat = u.angular.z + w_hat_dist(rng);
	double gamma = gamma_dist(rng);

	double v_w_ratio = v_hat/w_hat;

	double yaw = tf::getYaw(x->orientation);

	double x_prime = x->position.x - v_w_ratio*sin(yaw) + v_w_ratio*sin(yaw + w_hat*dt);
	double y_prime = x->position.y + v_w_ratio*cos(yaw) - v_w_ratio*cos(yaw + w_hat*dt);
	double yaw_prime = yaw + w_hat*dt + gamma*dt;

	x->position.x = x_prime;
	x->position.y = y_prime;
	x->orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, yaw_prime);
}

void initialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose_with_cov)
{
	ROS_INFO("Received a new pose");

	Eigen::VectorXd mean(3);
	Eigen::MatrixXd covar(3, 3);

	mean << pose_with_cov->pose.pose.position.x,
		pose_with_cov->pose.pose.position.y,
		tf::getYaw(pose_with_cov->pose.pose.orientation);
	covar << pose_with_cov->pose.covariance[0], 0,                                  0,
		     0,                                 pose_with_cov->pose.covariance[7],  0,
		     0,                            0,                                       pose_with_cov->pose.covariance[35];

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

	Eigen::MatrixXd samples = (normTransform * Eigen::MatrixXd::NullaryExpr(3, NUM_POSES, randN)).colwise() + mean;

	ROS_INFO_STREAM("\nMean:\n" << mean << "\n");
	ROS_INFO_STREAM("\nCov:\n" << covar << "\n");

	// remove existing pose estimates
	pose_array.poses.clear();
	for(int i=0; i<NUM_POSES; i++)
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

void velocityCallback(const geometry_msgs::TwistStamped::ConstPtr& twist_msg)
{
	curr_vel_cmd = twist_msg;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "pose_array_test");
	ros::NodeHandle nh;

	// initialize alpha
	for(int i=0; i<6; i++)
		alpha[i] = .1;

	alpha[0] = .5;
	alpha[3] = .5;
	
	ros::Publisher pose_array_pub = nh.advertise<geometry_msgs::PoseArray>("pose_array_test", 1);
	//ros::Subscriber initial_pose_sub = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 10, &initialPoseCallback, ros::TransportHints().tcpNoDelay(false));
	ros::Subscriber velocity_sub = nh.subscribe<geometry_msgs::TwistStamped>("/cylbot/velocity", 1, &velocityCallback);


	pose_array.header.frame_id = "/map";

	geometry_msgs::Pose starting_pose;
	starting_pose.orientation.w = 1.0;
	for(int i=0; i<1000; i++)
	{
		pose_array.poses.push_back(starting_pose);
	}

	pose_array_pub.publish(pose_array);

	ros::Time last_time = ros::Time::now();
	ros::Rate loop_rate(100);
	while(ros::ok())
	{
		if(curr_vel_cmd != NULL)
		{
			// get the time delta
			ros::Time curr_time = ros::Time::now();
			ros::Duration dt = curr_time - last_time;
			last_time = curr_time;
			
			// skip when commands are at 0
			if(fabs(curr_vel_cmd->twist.linear.y) > 0.1 ||
			   fabs(curr_vel_cmd->twist.angular.z) > 0.1)
			{

				ROS_INFO_STREAM("Updating pose estimates with command\n" << curr_vel_cmd->twist);
				ROS_INFO_STREAM("\tdt: " << dt.toSec());

				// update the pose estimates with the probabilistic motion model
				for(geometry_msgs::PoseArray::_poses_type::iterator pose = pose_array.poses.begin();
					pose != pose_array.poses.end();
					pose++)
				{
					sampleVelocityMotionModel(curr_vel_cmd->twist, pose, dt.toSec(), alpha, rng);
				}
			}
		}

		pose_array.header.stamp = ros::Time::now();
		pose_array_pub.publish(pose_array);
		
		ros::spinOnce();
		loop_rate.sleep();
	}
	
}
