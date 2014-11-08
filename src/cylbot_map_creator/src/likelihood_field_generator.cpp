#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <cylbot_map_creator/LikelihoodField.h>
#include <cmath>
#include <fstream>
#include <utility>
#include <iostream>
#include <limits>
#include <yaml-cpp/yaml.h>

typedef std::pair<double, double> MapCell;

nav_msgs::OccupancyGrid::ConstPtr nav_map;

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& map_msg)
{
	nav_map = map_msg;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "likelihood_field_generator");

	ros::NodeHandle nh;

	ros::NodeHandle priv_nh("~");
	std::string output_file = "";
	priv_nh.getParam("output_file", output_file);

	ros::Subscriber map_sub = nh.subscribe<nav_msgs::OccupancyGrid>("/map", 1, &mapCallback);

	ros::Rate loop_rate(10);
	while(ros::ok() && nav_map == NULL)
	{
		ros::spinOnce();
		loop_rate.sleep();
	}

	// quit
	if(nav_map == NULL)
	{
		return 0;
	}

	std::cout << std::endl << "Computing the likelihood field" << std::endl << "------------------------------" << std::endl;

	cylbot_map_creator::LikelihoodField field;
	field.info = nav_map->info;

	// first get a list of every occupied cell
	std::cout << "Finding all of the occupied cells" << std::endl;
	std::vector<MapCell> occupiedCells;
	for(int i=0; i<nav_map->info.width; i++)
	{
		for(int j=0; j<nav_map->info.height; j++)
		{
			int occ = nav_map->data[j*nav_map->info.width + i];
			if(occ > .5)
				occupiedCells.push_back(std::make_pair(i*nav_map->info.resolution, j*nav_map->info.resolution));
		}
	}

	field.data.resize(nav_map->data.size());
	std::cout << "Calculating minimum distances" << std::endl;
	for(int i=0; i<nav_map->info.width; i++)
	{
		for(int j=0; j<nav_map->info.height; j++)
		{
			int index = j*nav_map->info.width + i;
			int progressInterval = (int)(nav_map->data.size()*.1);
			if(index % progressInterval == 0)
				std::cout << 100*((double)index)/((double)nav_map->data.size()) << "%" << std::endl;

			double x = i*nav_map->info.resolution;
			double y = j*nav_map->info.resolution;
			double minDist = std::numeric_limits<double>::max();
			for(std::vector<MapCell>::const_iterator cell = occupiedCells.begin();
				cell != occupiedCells.end();
				cell++)
			{
				double x_diff = x - cell->first;
				double y_diff = y - cell->second;
				double dist = sqrt(x_diff*x_diff + y_diff*y_diff);

				if(dist < minDist)
					minDist = dist;
			}

			field.data.push_back(minDist);
		}
	}

	std::cout << "Finished. Saving data" << std::endl;

	YAML::Emitter emitter;
	emitter << YAML::BeginMap;
	emitter << YAML::Key << "info";
	emitter << YAML::Value << YAML::BeginMap;
	emitter << YAML::Key << "resolution" << YAML::Value << nav_map->info.resolution;
	emitter << YAML::Key << "width" << YAML::Value << nav_map->info.width;
	emitter << YAML::Key << "height" << YAML::Value << nav_map->info.height;
	emitter << YAML::Key << "origin" << YAML::BeginMap << YAML::Key << "position" << YAML::Value << YAML::BeginMap;
	emitter << YAML::Key << "x" << YAML::Value << nav_map->info.origin.position.x;
	emitter << YAML::Key << "y" << YAML::Value << nav_map->info.origin.position.y;
	emitter << YAML::Key << "z" << YAML::Value << nav_map->info.origin.position.z;
	emitter << YAML::EndMap;
	emitter << YAML::Key << "orientation" << YAML::Value << YAML::BeginMap;
	emitter << YAML::Key << "x" << YAML::Value << nav_map->info.origin.orientation.x;
	emitter << YAML::Key << "y" << YAML::Value << nav_map->info.origin.orientation.y;
	emitter << YAML::Key << "z" << YAML::Value << nav_map->info.origin.orientation.z;
	emitter << YAML::Key << "w" << YAML::Value << nav_map->info.origin.orientation.w;
	emitter << YAML::EndMap << YAML::EndMap << YAML::EndMap;
	emitter << YAML::Key << "data" << YAML::Value << YAML::Flow << field.data;
	emitter << YAML::EndMap;

	std::ofstream yaml_file(output_file.c_str());
	yaml_file << emitter.c_str();
}
