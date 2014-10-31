#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <laser_geometry/laser_geometry.h>
#include <tf/transform_listener.h>

laser_geometry::LaserProjection laser_projector;
tf::TransformListener* tf_listener;
ros::Publisher laser_cloud_pub;

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_in)
{
	if(!tf_listener->waitForTransform(
		   scan_in->header.frame_id,
		   "/base_link",
		   scan_in->header.stamp + ros::Duration().fromSec(scan_in->ranges.size()*scan_in->time_increment),
		   ros::Duration(1.0))){
		return; // if a transform isn't found within one second give up
	}

	sensor_msgs::PointCloud2 cloud_out;
	laser_projector.transformLaserScanToPointCloud("/base_link", *scan_in, cloud_out, *tf_listener);

	laser_cloud_pub.publish(cloud_out);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "hokuyo_laser_to_pointcloud");

	ros::NodeHandle nh;

	tf_listener = new tf::TransformListener();
	
	laser_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/laser/points", 1);

	ros::Subscriber scan_sub = nh.subscribe<sensor_msgs::LaserScan>("laser/scan", 10, &scanCallback);

	ros::spin();

	delete tf_listener;
}
