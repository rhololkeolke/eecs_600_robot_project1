#include <ros/ros.h>
#include <cylbot_map_creator/LikelihoodField.h>
#include <yaml-cpp/yaml.h>
#include <vector>
#include <string>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "likelihood_field_server");

	ros::NodeHandle nh;

	ros::NodeHandle priv_nh("~");
	std::string field_file = "";
	priv_nh.getParam("field_file", field_file);

	ros::Publisher field_pub = nh.advertise<cylbot_map_creator::LikelihoodField>("/likelihood_field", 1, true);

	cylbot_map_creator::LikelihoodField field_msg;
	field_msg.header.frame_id = "/map";
	field_msg.header.stamp = ros::Time::now();
	
	YAML::Node yaml_file = YAML::LoadFile(field_file.c_str());
	field_msg.info.resolution = yaml_file["info"]["resolution"].as<double>();
	field_msg.info.width = yaml_file["info"]["width"].as<int>();
	field_msg.info.height = yaml_file["info"]["height"].as<int>();
	field_msg.info.origin.position.x = yaml_file["info"]["origin"]["position"]["x"].as<double>();
	field_msg.info.origin.position.y = yaml_file["info"]["origin"]["position"]["y"].as<double>();
	field_msg.info.origin.position.z = yaml_file["info"]["origin"]["position"]["z"].as<double>();
	field_msg.info.origin.orientation.x = yaml_file["info"]["origin"]["orientation"]["x"].as<double>();
	field_msg.info.origin.orientation.y = yaml_file["info"]["origin"]["orientation"]["y"].as<double>();
	field_msg.info.origin.orientation.z = yaml_file["info"]["origin"]["orientation"]["z"].as<double>();
	field_msg.info.origin.orientation.w = yaml_file["info"]["origin"]["orientation"]["w"].as<double>();
	field_msg.data = yaml_file["data"].as<std::vector<double> >();

	field_pub.publish(field_msg);

	ros::spin();
}
