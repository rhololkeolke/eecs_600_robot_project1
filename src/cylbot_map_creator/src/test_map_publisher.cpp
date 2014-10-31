#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "test_map_publisher");

	ros::NodeHandle nh;
	ros::Publisher map_pub = nh.advertise<nav_msgs::OccupancyGrid>("/test_map", 1, true);

	nav_msgs::OccupancyGrid test_map;
	// fill out static parts of the header
	test_map.header.frame_id = "/map";

	// fill out the meta-data
	test_map.info.map_load_time = ros::Time::now();
	test_map.info.resolution = 0.05f;
	test_map.info.width = 1000;
	test_map.info.height = 1000;
	test_map.info.origin.position.x = -(test_map.info.resolution*test_map.info.width)/2.0;
	test_map.info.origin.position.y = -(test_map.info.resolution*test_map.info.height)/2.0;
	test_map.info.origin.orientation.w = 1.0;

	for(int i=0; i<test_map.info.height; i++)
	{
		for(int j=0; j<test_map.info.width; j++)
		{
			switch((test_map.data.size() + 1) % 3)
			{
			case 0:
				test_map.data.push_back(0);
				break;
			case 1:
				test_map.data.push_back(100);
				break;
			case 2:
				test_map.data.push_back(-1);
				break;
			}
		}
	}
	
	ros::Rate loop_rate(10.0);
	while(ros::ok())
	{
		test_map.header.stamp = ros::Time::now();
		map_pub.publish(test_map);
		loop_rate.sleep();
	}

}
