#include <ros/ros.h>
#include <footstep_plan/FootStepPlan.h>
#include <footstep_plan/FootStepPose.h>
#include <footstep_plan/roadmap.h>
#include <footstep_plan/yaml_conversions.h>
#include <yaml-cpp/yaml.h>
#include <string>
#include <tf/transform_datatypes.h>
#include <vector>
#include <boost/random/uniform_real_distribution.hpp>
#include <boost/bind.hpp>
#include <boost/random.hpp>
#include <nav_msgs/OccupancyGrid.h>
#include <utility>
#include <cmath>
#include <fstream>
#include <flann/flann.hpp>
#include <queue>

#define MIN_STRIDE 0.05
#define MAX_STRIDE 0.2

typedef struct MapBounds_ {
	std::pair<double, double> x_bounds;
	std::pair<double, double> y_bounds;
} MapBounds;

void mapSizeCallback(const nav_msgs::OccupancyGrid::ConstPtr& map,
					 ros::Subscriber* map_sub,
					 MapBounds* bounds,
					 nav_msgs::OccupancyGrid::ConstPtr* saved_map)
{
	bounds->x_bounds.first = map->info.origin.position.x;
	bounds->x_bounds.second = map->info.width*map->info.resolution + map->info.origin.position.x;
	bounds->y_bounds.first = map->info.origin.position.y;
	bounds->y_bounds.second = map->info.width*map->info.resolution + map->info.origin.position.y;

	*saved_map = map;
	
	map_sub->shutdown();
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "generate_prm");

	ros::NodeHandle nh;

	nav_msgs::OccupancyGrid::ConstPtr map;
	MapBounds map_bounds;
	ros::Subscriber map_sub;
	map_sub = nh.subscribe<nav_msgs::OccupancyGrid>("/map", 1, boost::bind(mapSizeCallback,
																		   _1,
																		   &map_sub,
																		   &map_bounds,
																		   &map));

	ros::NodeHandle priv_nh("~");
	std::string output_file;
	priv_nh.getParam("output_file", output_file);

	int num_nodes;
	priv_nh.param<int>("num_nodes", num_nodes, 10);

	int num_nn;
	priv_nh.param<int>("num_nn", num_nn, 3);

	ROS_ASSERT_MSG(!output_file.empty(), "Must specify an output file location");
	ROS_ASSERT_MSG((num_nodes > 0), "num_nodes must be greater than 0");
	ROS_ASSERT_MSG((num_nn > 0), "num_nn must be greater than 0");

	ros::Rate loop_rate(10);
	while(ros::ok() && map == NULL)
	{
		ros::spinOnce();
		loop_rate.sleep();
	}

	ROS_INFO("Map x bounds: (%3.3f, %3.3f)", map_bounds.x_bounds.first, map_bounds.x_bounds.second);
	ROS_INFO("Map y bounds: (%3.3f, %3.3f)", map_bounds.y_bounds.first, map_bounds.y_bounds.second);

	ROS_INFO("Constructing a KD-Tree of the occupied map cells");
	std::vector<float> rawOccupiedLocations;
	for(int j=0; j<map->info.height; j++)
	{
		for(int i=0; i<map->info.width; i++)
		{
			// don't add the point if its unoccupied
			if(map->data[j*map->info.width + i] < 50)
				continue;
			
			float x = i*map->info.resolution + map->info.origin.position.x;
			float y = j*map->info.resolution + map->info.origin.position.y;

			rawOccupiedLocations.push_back(x);
			rawOccupiedLocations.push_back(y);
		}
	}

	flann::Matrix<float> occupiedLocations(&rawOccupiedLocations[0], (int)(rawOccupiedLocations.size()/2.0), 2);
	flann::Index<flann::L2<float> > map_index(occupiedLocations, flann::KDTreeIndexParams(4));
	map_index.buildIndex();

	boost::mt19937 rng;
	boost::random::uniform_real_distribution<> x_dist(map_bounds.x_bounds.first,
													  map_bounds.x_bounds.second);
	boost::random::uniform_real_distribution<> y_dist(map_bounds.y_bounds.first,
													  map_bounds.y_bounds.second);
	boost::random::uniform_real_distribution<> yaw_dist(0.0, 2*M_PI);

	boost::variate_generator<boost::mt19937, boost::random::uniform_real_distribution<> > x_rand(rng, x_dist);
	boost::variate_generator<boost::mt19937, boost::random::uniform_real_distribution<> > y_rand(rng, y_dist);
	boost::variate_generator<boost::mt19937, boost::random::uniform_real_distribution<> > yaw_rand(rng, yaw_dist);	

	ROS_INFO("Generating %d nodes", num_nodes);
	std::vector<footstep_plan::FootStepPose> nodes;
	while(nodes.size() < num_nodes)
	{
		double x_center = x_rand();
		x_rand();
		double y_center = y_rand();
		x_rand();
		double yaw = yaw_rand();

		tf::Transform pose_transform;
		pose_transform.setOrigin(tf::Vector3(x_center, y_center, 0));
		pose_transform.setRotation(tf::createQuaternionFromYaw(yaw));

		tf::Vector3 left_foot_point = pose_transform(tf::Vector3(0, .15, 0));
		tf::Vector3 right_foot_point = pose_transform(tf::Vector3(0, -.15, 0));

		footstep_plan::FootStepPose footstep_pose;
		tf::pointTFToMsg(left_foot_point, footstep_pose.left_foot.position);
		tf::pointTFToMsg(right_foot_point, footstep_pose.right_foot.position);
		footstep_pose.left_foot.orientation = tf::createQuaternionMsgFromYaw(yaw);
		footstep_pose.right_foot.orientation = footstep_pose.left_foot.orientation;

		// TODO improve this so foots will work over gaps
		// for now just check if any occupied points within radius of .35m
		float rawQuery[2];
		rawQuery[0] = x_center;
		rawQuery[1] = y_center;
		flann::Matrix<float> query(rawQuery, 1, 2);
		flann::Matrix<int> indices(new int[1], 1, 1);
		flann::Matrix<float> dists(new float[1], 1, 1);

		map_index.knnSearch(query, indices, dists, 1, flann::SearchParams(128));

		// there is a collision
		if(dists[0][0] <= .35*.35)
		{
			delete[] indices.ptr();
			delete[] dists.ptr();
			ROS_DEBUG("Collision!");
			continue;
		}

		nodes.push_back(footstep_pose);
		ROS_DEBUG("original node[%d] (%f, %f)", (int)(nodes.size()-1), x_center, y_center);

		delete[] indices.ptr();
		delete[] dists.ptr();
	}

	ROS_INFO("Constructing node KD-tree");
	float* rawNodeDataset = new float[nodes.size()*2];
	float* rawNodeQueries = new float[nodes.size()*2];
	for(int i=0; i<2*nodes.size(); i+=2)
	{
		float avg_x = (nodes[i/2].left_foot.position.x - nodes[i/2].right_foot.position.x)/2.0 + nodes[i/2].right_foot.position.x;
		float avg_y = (nodes[i/2].left_foot.position.y - nodes[i/2].right_foot.position.y)/2.0 + nodes[i/2].right_foot.position.y;
		//float avg_yaw = (tf::getYaw(nodes[i].left_foot.orientation) + tf::getYaw(nodes[i].right_foot.orientation))/2.0;

		ROS_DEBUG("node[%d] avg_x: %f avg_y: %f", i/2, avg_x, avg_y);
		
		rawNodeDataset[i] = avg_x;
		rawNodeDataset[i+1] = avg_y;
		//rawNodeDataset[i+2] = sin(avg_yaw);
		//rawNodeDataset[i+3] = cos(avg_yaw);
		rawNodeQueries[i] = avg_x;
		rawNodeQueries[i+1] = avg_y;
		//rawNodeQueries[i+2] = sin(avg_yaw);
		//rawNodeQueries[i+3] = cos(avg_yaw);
	}

	ROS_INFO("Performing %d nearest neighbors search", num_nn);
	flann::Matrix<float> nodeDataset(rawNodeDataset, nodes.size(), 2);
	flann::Index<flann::L2<float> > nodeIndex(nodeDataset, flann::KDTreeIndexParams(4));
	nodeIndex.buildIndex();

	flann::Matrix<float> nodeQueries(rawNodeQueries, nodes.size(), 2);
	// NN search will return itself and num_nn-1 others. We don't care about edges between itself
	// so bump the nn amount up by one.
	flann::Matrix<int> nodeIndices(new int[nodeQueries.rows*(num_nn+1)], nodeQueries.rows, num_nn+1);
	flann::Matrix<float> nodeDists(new float[nodeQueries.rows*(num_nn+1)], nodeQueries.rows, num_nn+1);

	nodeIndex.knnSearch(nodeQueries, nodeIndices, nodeDists, num_nn+1, flann::SearchParams(128));

	ROS_INFO("populating roadmap nodes list");
	footstep_plan::Roadmap roadmap;
	for(int i=0; i<nodes.size(); i++)
	{
		roadmap.nodes[i] = nodes[i];
	}

	int progressInterval = (int)(nodes.size()*.1);
	for(int i=0; i<nodes.size(); i++)
	{
		if(i % progressInterval == 0)
			ROS_INFO("%d%% complete", i*10/progressInterval);

		for(int j=1; j<num_nn+1; j++)
		{
			int nn_index = nodeIndices[i][j];
			ROS_DEBUG("node[%d]->node[%d]", i, nn_index);
			
			footstep_plan::RoadmapEdges::iterator outer_edge = roadmap.edges.find(i);
			if(outer_edge != roadmap.edges.end())
			{
				footstep_plan::RoadmapInnerEdges::const_iterator inner_edge = outer_edge->second.find(nn_index);
				if(inner_edge != outer_edge->second.end())
				{
					ROS_INFO("Edge exists between %d and %d", i, nn_index);
					continue;
				}
			}

			tf::Vector3 start_point(nodeDataset[i][0], nodeDataset[i][1], 0);
			tf::Vector3 end_point(nodeDataset[nn_index][0], nodeDataset[nn_index][0], 0);
			tf::Vector3 path = end_point - start_point;
			tf::Vector3 path_dir = (end_point - start_point).normalize();
			float path_length = path.length();

			ROS_DEBUG("start_point (%f, %f)", start_point.getX(), start_point.getY());
			ROS_DEBUG("end_point:  (%f, %f)", end_point.getX(), end_point.getY());

			if(path_length <= MAX_STRIDE)
			{
				// found an edge
				// in this case the two nodes can be reached in a single step
				// so the plan will be empty
				ROS_INFO("Close enough for empty plan edge");
				footstep_plan::FootStepPlan plan;
				if(outer_edge == roadmap.edges.end())
				{
					footstep_plan::RoadmapInnerEdges inner_edges;
					inner_edges[nn_index] = plan;
					roadmap.edges[i] = inner_edges;
				}
				else
				{
					footstep_plan::RoadmapInnerEdges::iterator inner_edge = outer_edge->second.find(nn_index);
					if(inner_edge != outer_edge->second.end())
					{
						outer_edge->second[nn_index] = plan;
					}
				}

				continue;
			}

			// the end points of the line segment to check the midpoint of
			std::queue<std::pair<float, float> > line_segments;
			line_segments.push(std::make_pair(0, 1.0));
			bool collisions = false;
			flann::Matrix<int> indices(new int[1], 1, 1);
			flann::Matrix<float> dists(new float[1], 1, 1);
			while(!line_segments.empty())
			{
			 	std::pair<float, float> line_segment = line_segments.front();
			 	if((line_segment.second - line_segment.first)*path_length < .1)
			 		break;

			 	// get point along path using linear interpolation
			 	float lambda = line_segment.first + (line_segment.second - line_segment.first)/2.0;
			 	float rawQuery[2];
			 	rawQuery[0] = start_point.getX()*(1.0 - lambda) + end_point.getX()*lambda;
			 	rawQuery[1] = start_point.getY()*(1.0 - lambda) + end_point.getY()*lambda;

			    flann::Matrix<float> query(rawQuery, 1, 2);

				map_index.knnSearch(query, indices, dists, 1, flann::SearchParams(128));

				// there is a collision so stop checking this path
				ROS_DEBUG("map_dists: %f", dists[0][0]);
				if(dists[0][0] < .35*.35)
				{
					collisions = true;
					break;
				}

				line_segments.push(std::make_pair(line_segment.first, lambda + line_segment.first));
				line_segments.push(std::make_pair(line_segment.first + lambda, line_segment.second));

				line_segments.pop();
			}
			 
			delete[] indices.ptr();
			delete[] dists.ptr();

			if(collisions)
			{
				ROS_INFO("Found a collision for %d closest neighbor %d of %d", j, nn_index, i);
				continue;
			}

			// build the plan
			footstep_plan::FootStepPlan plan;
			atlas_msgs::AtlasBehaviorStepData last_step;
			last_step.step_index = 0;
			last_step.foot_index = 0;
			last_step.duration = .63;
			tf::Vector3 curr_pos = start_point + (MAX_STRIDE/2.0)*path_dir;
			tf::Transform path_transform;
			path_transform.setOrigin(curr_pos);
			path_transform.setRotation(tf::createQuaternionFromYaw(atan2(path_dir.getY(), path_dir.getX())));
			tf::Vector3 foot_pos = path_transform(tf::Vector3(0, .15, 0));
			last_step.pose.position.x = foot_pos.getX();
			last_step.pose.position.y = foot_pos.getY();
			last_step.pose.position.z = 0;
			last_step.pose.orientation = tf::createQuaternionMsgFromYaw(atan2(path_dir.getY(), path_dir.getX()));
			plan.steps.push_back(last_step);
			while((end_point - tf::Vector3(last_step.pose.position.x, last_step.pose.position.y, 0)).length() > MAX_STRIDE)
			{
				last_step.step_index++;
				last_step.foot_index = last_step.step_index % 2;
				curr_pos = curr_pos + (MAX_STRIDE/2.0)*path_dir;
				path_transform.setOrigin(curr_pos);
				if(last_step.foot_index == 0)
					foot_pos = path_transform(tf::Vector3(0, .15, 0));
				else
					foot_pos = path_transform(tf::Vector3(0, -.15, 0));
				last_step.pose.position.x = foot_pos.getX();
				last_step.pose.position.y = foot_pos.getY();
				plan.steps.push_back(last_step);
			}

			if(outer_edge == roadmap.edges.end())
			{
				footstep_plan::RoadmapInnerEdges inner_edges;
				inner_edges[nn_index] = plan;
				roadmap.edges[i] = inner_edges;
			}
			else
			{
				footstep_plan::RoadmapInnerEdges::iterator inner_edge = outer_edge->second.find(nn_index);
				if(inner_edge != outer_edge->second.end())
				{
					outer_edge->second[nn_index] = plan;
				}
			}
		}
	}
	
	YAML::Emitter emitter;
	emitter << roadmap;

	std::ofstream yaml_file(output_file.c_str());
	yaml_file << emitter.c_str();

	return 0;

}
