#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/Marker.h>
#include <boost/bind.hpp>

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& map_msg,
				 ros::Publisher* marker_pub)
{
	visualization_msgs::Marker markers;
	markers.header.frame_id = "/map";
	markers.header.stamp = ros::Time::now();
	markers.ns = "map_points";
	markers.id = 0;
	markers.type = visualization_msgs::Marker::POINTS;
	markers.action = visualization_msgs::Marker::ADD;
	markers.pose.orientation.w = 1.0;
	markers.scale.x = .1;
	markers.scale.y = .1;
	markers.scale.z = .1;
	markers.color.a = 1.0;
	markers.color.r = 1.0;
	markers.color.g = 0.0;
	markers.color.b = 0.0;

	for(int j=0; j<map_msg->info.height; j++)
	{
		for(int i=0; i<map_msg->info.width; i++)
		{
			if(map_msg->data[j*map_msg->info.width + i] < 50)
				continue;

			float x = i*map_msg->info.resolution + map_msg->info.origin.position.x;
			float y = j*map_msg->info.resolution + map_msg->info.origin.position.y;

			geometry_msgs::Point new_point;
			new_point.x = x;
			new_point.y = y;
			new_point.z = .1;

			markers.points.push_back(new_point);
			markers.colors.push_back(markers.color);
		}
	}

	marker_pub->publish(markers);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "map_transform_test");

	ros::NodeHandle nh;

	ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("/map_transform_test", 1, true);
	
	ros::Subscriber map_sub = nh.subscribe<nav_msgs::OccupancyGrid>("/map", 1,
																	boost::bind(mapCallback,
																				_1,
																				&marker_pub));

	ros::spin();
}
