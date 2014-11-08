#include <ros/ros.h>
#include <cylbot_map_creator/LikelihoodField.h>
#include <nav_msgs/OccupancyGrid.h>
#include <vector>
#include <limits>

ros::Publisher map_pub;
nav_msgs::OccupancyGrid nav_map;

void fieldCallback(const cylbot_map_creator::LikelihoodField::ConstPtr& field)
{
	nav_map.info = field->info;
	nav_map.data.clear();
	
	double max_distance = -std::numeric_limits<double>::max();
	for(cylbot_map_creator::LikelihoodField::_data_type::const_iterator distance = field->data.begin();
		distance != field->data.end();
		distance++)
	{
		if(*distance > max_distance)
			max_distance = *distance;
	}

	ROS_INFO_STREAM("max distance:" << max_distance);

	// normalize and multiply by 100
	for(cylbot_map_creator::LikelihoodField::_data_type::const_iterator distance = field->data.begin();
		distance != field->data.end();
		distance++)
	{
		int map_val = (int)(100*(*distance/max_distance));
		nav_map.data.push_back(map_val);
	}

	ROS_INFO_STREAM("field size:" << field->data.size());
	ROS_INFO_STREAM("map size:" << nav_map.data.size());

	nav_map.header.stamp = ros::Time::now();
	nav_map.header.frame_id="/map";
	map_pub.publish(nav_map);

}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "publish_likelihood_field_as_map");

	ros::NodeHandle nh;

	ros::Subscriber field_sub = nh.subscribe<cylbot_map_creator::LikelihoodField>("/likelihood_field", 1, &fieldCallback);
	map_pub = nh.advertise<nav_msgs::OccupancyGrid>("/likelihood_map", 1, true);

	ros::spin();
}
