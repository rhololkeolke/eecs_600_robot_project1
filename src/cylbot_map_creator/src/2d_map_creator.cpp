#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/OccupancyGrid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <tf/transform_listener.h>
#include <pcl_ros/transforms.h>
#include <cmath>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloudXYZ;

nav_msgs::OccupancyGrid nav_map;
ros::Publisher map_pub;
tf::TransformListener* tf_listener;
bool map_changed;

int x_origin, y_origin;

inline int round (const float a) { return int (a + 0.5); }

void setCellOccupancy(const int x, const int y, const int value)
{
	// only include this point if it falls within the map grid
	if(x < nav_map.info.width && x >= 0 &&
	   y < nav_map.info.height && y >= 0)
	{
		nav_map.data[y*nav_map.info.width + x] = value;
	}
}

void setCellOccupancy(const float x, const float y, const int value)
{
	int x_coord = (int)(x/nav_map.info.resolution) + x_origin;
	int y_coord = (int)(y/nav_map.info.resolution) + y_origin;

	setCellOccupancy(x_coord, y_coord, value);
}

int getCellOccupancy(const int x, const int y)
{
	if(x < nav_map.info.width && x >= 0 &&
	   y < nav_map.info.height && y >= 0)
	{
		return nav_map.data[y*nav_map.info.width + x];
	}

	return 50;
}

int getCellOccupancy(const float x, const float y)
{
	int x_coord = (int)(x/nav_map.info.resolution) + x_origin;
	int y_coord = (int)(y/nav_map.info.resolution) + y_origin;

	return getCellOccupancy(x_coord, y_coord);
}

void beamDDA(const tf::Vector3& beamStart, const tf::Vector3& beamEnd)
{
	int dx = (int)((beamEnd.getX() - beamStart.getX())/nav_map.info.resolution);
	int dy = (int)((beamEnd.getY() - beamStart.getY())/nav_map.info.resolution);
	int steps;

	float xIncrement, yIncrement;
	float x = beamStart.getX()/nav_map.info.resolution;
	float y = beamStart.getY()/nav_map.info.resolution;

	if(fabs(dx) > fabs(dy))
		steps = fabs(dx);
	else
		steps = fabs(dy);
	xIncrement = ((float)dx)/((float)steps);
	yIncrement = ((float)dy)/((float)steps);

/*	ROS_INFO("%3.3f, %3.3f, %3.3f, %3.3f, %d, %d, %d, %3.3f, %3.3f",
			 beamStart.getX(), beamStart.getY(),
			 beamEnd.getX(), beamEnd.getY(),
			 dx, dy, steps,
			 xIncrement, yIncrement);*/

	setCellOccupancy(x, y, 0);
	for(int k=0; k<steps-1; k++)
	{
		x += xIncrement;
		y += yIncrement;
		int new_value = getCellOccupancy(round(x)+x_origin, round(y)+y_origin) - 5;
		if(new_value < 0)
			new_value = 0;
		setCellOccupancy(round(x)+x_origin, round(y)+y_origin, new_value);
	}
}

void laserCallback(const sensor_msgs::PointCloud2::ConstPtr& ros_cloud)
{
	sensor_msgs::PointCloud2 map_ros_cloud;
	if(!tf_listener->waitForTransform(
		   "/map",
		   "/base_link",
		   ros_cloud->header.stamp,
		   ros::Duration(1.0)))
	{
		ROS_INFO("Transform from /map to /base_link failed");
		return; // if a transform isn't found within one second give up
	}
	pcl_ros::transformPointCloud("/map", *ros_cloud, map_ros_cloud, *tf_listener);
	// get the start point of the laser beam
	tf::StampedTransform laser_transform;
	if(!tf_listener->waitForTransform(
		   "/map",
		   "/head_hokuyo_frame",
		   ros_cloud->header.stamp,
		   ros::Duration(1.0)))
	{
		ROS_INFO("Transform from /map to /head_hokuyo_frame failed");
		return;
	}
	tf_listener->lookupTransform("/map", "/head_hokuyo_frame", ros_cloud->header.stamp, laser_transform);
	tf::Vector3 beam_start = laser_transform.getOrigin();
	beam_start.setZ(0);
	
	// convert to the PCL format
	PointCloudXYZ pcl_cloud;
	pcl::fromROSMsg(map_ros_cloud, pcl_cloud);

	for(pcl::PointCloud<pcl::PointXYZ>::const_iterator point = pcl_cloud.points.begin();
		point != pcl_cloud.points.end();
		point++)
	{
		if(point->z > 1.2 || point->z < .8)
			continue; // ignore points outside of the scan plane

		// get the vector in the map plane
		tf::Vector3 beam_end(point->x, point->y, 0);

		beamDDA(beam_start, beam_end);

		double beam_length = beam_end.distance(beam_start);
		// ignore the max distance results and results that come from
		// within the bounding cylinder of the robot
		if(fabs(beam_length - 30.0) < .03 || fabs(beam_length) < .5)
			continue; // skip points that are near the max sensor range
		
		int new_value = getCellOccupancy(point->x, point->y) + 25;
		if(new_value > 100)
			new_value = 100;
		setCellOccupancy(point->x, point->y, new_value);
	}

	map_changed = true;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "map_creator_2d");
	ros::NodeHandle nh;

	ros::NodeHandle priv_nh("~");
	double map_resolution, x_offset, y_offset;
	int map_width, map_height;
	priv_nh.param<double>("resolution", map_resolution, 0.05);
	priv_nh.param<int>("width", map_width, 1000);
	priv_nh.param<int>("height", map_height, 1000);
	priv_nh.param<double>("x_offset", x_offset, .5);
	priv_nh.param<double>("y_offset", y_offset, .5);

	ROS_INFO("Map Resolution %3.3f", map_resolution);
	ROS_INFO("Map Size (%d, %d)", map_width, map_height);
	ROS_INFO("Map Origin Offset (%3.3f, %3.3f)", x_offset, y_offset);

	if(x_offset < 0 || x_offset > 1)
	{
		ROS_ERROR("Invalid x_offset. Must be in range [0, 1.0]");
		return 1;
	}

	if(y_offset < 0 || y_offset > 1)
	{
		ROS_ERROR("Invalid y_offset. Must be in range [0, 1.0]");
		return 1;
	}
	

	tf_listener = new tf::TransformListener();

	// set the nav_map frame
	nav_map.header.frame_id = "/map";

	// fill out the meta-data
	nav_map.info.map_load_time = ros::Time::now();
	nav_map.info.resolution = map_resolution;
	nav_map.info.width = map_width;
	nav_map.info.height = map_height;
	nav_map.info.origin.position.x = -nav_map.info.resolution*nav_map.info.width*x_offset;
	nav_map.info.origin.position.y = -nav_map.info.resolution*nav_map.info.height*y_offset;
	nav_map.info.origin.orientation.w = 1.0;

	// initialize to all unknown
	for(int i=0; i<nav_map.info.height; i++)
	{
		for(int j=0; j<nav_map.info.width; j++)
		{
			nav_map.data.push_back(50);
		}
	}

	// get the origin coordinates in the map grid
	x_origin = (int)(nav_map.info.width*x_offset);
	y_origin = (int)(nav_map.info.height*y_offset);

	// now that the datastructure is initialized set up the ROS stuff
	map_pub = nh.advertise<nav_msgs::OccupancyGrid>("/map", 1, true);
	ros::Subscriber laser_sub = nh.subscribe<sensor_msgs::PointCloud2>("laser/points", 10, &laserCallback);

	ros::Rate loop_rate(10);
	while(ros::ok())
	{
		if(map_changed)
		{
			nav_map.header.stamp = ros::Time::now();
			map_pub.publish(nav_map);
			map_changed = false;
		}

		ros::spinOnce();
		loop_rate.sleep();
	}

	delete tf_listener;
}
