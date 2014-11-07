#include <gtest/gtest.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Pose.h>

TEST(Transform, transformPoint)
{
	geometry_msgs::Pose pose;
	pose.position.x = 10.0;
	pose.orientation.w = 1.0;

	tf::Transform transform;
	transform.setOrigin(tf::Vector3(pose.position.x, pose.position.y, pose.position.z));

	tf::Quaternion pose_rotation;
	tf::quaternionMsgToTF(pose.orientation, pose_rotation);
	
	transform.setRotation(pose_rotation);

	{
		tf::Vector3 point(0,0,0);

		tf::Vector3 transformed_point = transform(point);

		ASSERT_EQ(transformed_point.getX(), 10.0);
		ASSERT_EQ(transformed_point.getY(), 0.0);
		ASSERT_EQ(transformed_point.getZ(), 0.0);
	}

	{
		tf::Vector3 point(10.0, 10.0, 0);

		tf::Vector3 transformed_point = transform(point);

		ASSERT_EQ(transformed_point.getX(), 20.0);
		ASSERT_EQ(transformed_point.getY(), 10.0);
		ASSERT_EQ(transformed_point.getZ(), 0.0);
	}
	
}

int main(int argc, char** argv)
{
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
