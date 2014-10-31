#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/OccupancyGrid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <tf/transform_listener.h>
#include <pcl_ros/transforms.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloudXYZ;

nav_msgs::OccupancyGrid nav_map;
ros::Publisher map_pub;
tf::TransformListener* tf_listener;

int x_origin, y_origin;

void laserCallback(const sensor_msgs::PointCloud2::ConstPtr& ros_cloud)
{
	sensor_msgs::PointCloud2 map_ros_cloud;
	if(!tf_listener->waitForTransform(
		   "/map",
		   "/base_link",
		   ros_cloud->header.stamp,
		   ros::Duration(1.0)))
	{
		ROS_INFO("Transform failure");
		return; // if a transform isn't found within one second give up
	}
	pcl_ros::transformPointCloud("/map", *ros_cloud, map_ros_cloud, *tf_listener);
	
	// convert to the PCL format
	PointCloudXYZ pcl_cloud;
	pcl::fromROSMsg(map_ros_cloud, pcl_cloud);

	for(pcl::PointCloud<pcl::PointXYZ>::const_iterator point = pcl_cloud.points.begin();
		point != pcl_cloud.points.end();
		point++)
	{
		if(point->z > 1.2 || point->z < .8)
			continue; // ignore points outside of the scan plane
		
		// project the points orthogonally down and bin the x,y coordinates
		int x_coord = (int)(point->x/nav_map.info.resolution) + x_origin;
		int y_coord = (int)(point->y/nav_map.info.resolution) + y_origin;

		// only include this point if it falls within the map grid
		if(x_coord < nav_map.info.width && x_coord >= 0 &&
		   y_coord < nav_map.info.height && y_coord >= 0)
		{
			// this will have to be smarter, but as a first pass
			// just add this to the map as 100% certain occupied
			nav_map.data[y_coord*nav_map.info.width + x_coord] = 100;
		}
	}
	
	nav_map.header.stamp = ros::Time::now();
	map_pub.publish(nav_map);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "map_creator_2d");
	ros::NodeHandle nh;

	tf_listener = new tf::TransformListener();

	// set the nav_map frame
	nav_map.header.frame_id = "/map";

	// fill out the meta-data
	nav_map.info.map_load_time = ros::Time::now();
	nav_map.info.resolution = 0.05f;
	nav_map.info.width = 1000;
	nav_map.info.height = 1000;
	nav_map.info.origin.position.x = -nav_map.info.resolution*(nav_map.info.width/2.0);
	nav_map.info.origin.position.y = -nav_map.info.resolution*(nav_map.info.height/2.0);
	nav_map.info.origin.orientation.w = 1.0;

	// initialize to all unknown
	for(int i=0; i<nav_map.info.height; i++)
	{
		for(int j=0; j<nav_map.info.width; j++)
		{
			nav_map.data.push_back(-1);
		}
	}

	// get the origin coordinates in the map grid
	x_origin = (int)(nav_map.info.width/2.0);
	y_origin = (int)(nav_map.info.height/2.0);

	// now that the datastructure is initialized set up the ROS stuff
	map_pub = nh.advertise<nav_msgs::OccupancyGrid>("/map", 1, true);
	ros::Subscriber laser_sub = nh.subscribe<sensor_msgs::PointCloud2>("laser/points", 10, &laserCallback);

	ros::spin();

	delete tf_listener;
}
