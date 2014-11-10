#include <ros/ros.h>
#include <cylbot_map_creator/LikelihoodField.h>
#include <nav_msgs/OccupancyGrid.h>
#include <multisense_sensor_model/sensor_model.h>
#include <vector>
#include <limits>
#include <string>

using namespace multisense_sensor_model;

ros::Publisher map_pub;
nav_msgs::OccupancyGrid nav_map;
IntrinsicParams sensor_params;

void fieldCallback(const cylbot_map_creator::LikelihoodField::ConstPtr& field)
{
	nav_map.info = field->info;
	nav_map.data.clear();

	double max_prob = std::numeric_limits<double>::min();
	double min_prob = std::numeric_limits<double>::max();
	for(cylbot_map_creator::LikelihoodField::_data_type::const_iterator distance = field->data.begin();
		distance != field->data.end();
		distance++)
	{
		double prob = sensor_params.likelihoodProbability(*distance);
		if(prob > max_prob)
			max_prob = prob;
		if(prob < min_prob)
			min_prob = prob;
		nav_map.data.push_back(100-prob*100);
	}

	ROS_INFO_STREAM("max prob:" << max_prob);
	ROS_INFO_STREAM("min prob:" << min_prob);
	
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

	ros::NodeHandle priv_nh("~");
	std::string params_file = "";
	priv_nh.getParam("params_file", params_file);

	if(params_file.compare("") == 0)
	{
		ROS_WARN("Sensor params file not provided or not valid");
	}
	else
	{
		readIntrinsicParamsFromFile(params_file, &sensor_params);
	}
		

	ros::Subscriber field_sub = nh.subscribe<cylbot_map_creator::LikelihoodField>("/likelihood_field", 1, &fieldCallback);
	map_pub = nh.advertise<nav_msgs::OccupancyGrid>("/likelihood_map", 1, true);

	ros::spin();
}
