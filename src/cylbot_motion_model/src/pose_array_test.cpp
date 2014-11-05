#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
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

void initialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose_with_cov)
{
	std::cout << "TESTING" << std::endl;
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

int main(int argc, char** argv)
{
	ros::init(argc, argv, "pose_array_test");
	ros::NodeHandle nh;

	ros::Publisher pose_array_pub = nh.advertise<geometry_msgs::PoseArray>("pose_array_test", 1);
	ros::Subscriber initial_pose_sub = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 10, &initialPoseCallback, ros::TransportHints().tcpNoDelay(false));

	// random number generator
	boost::mt19937 rng;
	
	// creates a normal distribution with 0 mean and std dev of 1
	boost::normal_distribution<> norm_dist(0.0, 1.0);

	// create a method of sampling the distribution
	boost::variate_generator<boost::mt19937, boost::normal_distribution<> > norm_rng(rng, norm_dist);
	
	pose_array.header.frame_id = "/map";

	ros::Rate loop_rate(10);
	while(ros::ok())
	{
		ros::spinOnce();
		
		pose_array.header.stamp = ros::Time::now();
		pose_array_pub.publish(pose_array);
		loop_rate.sleep();
	}
	
}
