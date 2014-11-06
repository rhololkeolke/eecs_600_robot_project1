#include <cylbot_mcl/pose_cloud.h>
#include <gtest/gtest.h>

using namespace cylbot_mcl;

TEST(ConstructorTests, noInitialPosition)
{
	RobotModel model;
	PoseCloud2D pose_cloud(model);

	ASSERT_EQ(pose_cloud.getPoses().poses.size(), 0);
}

TEST(ConstructorTests, initialPosition)
{
	RobotModel model;
	geometry_msgs::PoseWithCovarianceStamped initial_pose;
	initial_pose.pose.pose.orientation.w = 1.0;
	initial_pose.pose.covariance[0] = initial_pose.pose.covariance[7] = initial_pose.pose.covariance[35] = 1.0;
	PoseCloud2D pose_cloud(model, initial_pose);

	ASSERT_EQ(pose_cloud.getPoses().poses.size(), 1000);
}

TEST(ResetCloud, resetCloud)
{
	RobotModel model;
	PoseCloud2D pose_cloud(model);

	ASSERT_EQ(pose_cloud.getPoses().poses.size(), 0);

	geometry_msgs::PoseWithCovarianceStamped initial_pose;
	initial_pose.pose.pose.orientation.w = 1.0;
	initial_pose.pose.covariance[0] = initial_pose.pose.covariance[7] = initial_pose.pose.covariance[35] = 1.0;

	pose_cloud.resetCloud(initial_pose);

	ASSERT_EQ(pose_cloud.getPoses().poses.size(), 1000);
}

int main(int argc, char** argv)
{
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
