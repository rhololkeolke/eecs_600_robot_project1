#ifndef FOOTSTEP_PLAN__YAML_CONVERSIONS_H_
#define FOOTSTEP_PLAN__YAML_CONVERSIONS_H_

#include <footstep_plan/roadmap.h>
#include <footstep_plan/FootStepPlan.h>
#include <yaml-cpp/yaml.h>
#include <sstream>
#include <string>

YAML::Emitter& operator<<(YAML::Emitter& out, const geometry_msgs::Pose& pose)
{
	out << YAML::BeginMap;
	out << YAML::Key << "position" << YAML::Value << YAML::BeginMap;
	out << YAML::Key << "x" << YAML::Value << pose.position.x;
	out << YAML::Key << "y" << YAML::Value << pose.position.y;
	out << YAML::Key << "z" << YAML::Value << pose.position.z;
	out << YAML::EndMap;
	out << YAML::Key << "orientation" << YAML::Value << YAML::BeginMap;
	out << YAML::Key << "x" << YAML::Value << pose.orientation.x;
	out << YAML::Key << "y" << YAML::Value << pose.orientation.y;
	out << YAML::Key << "z" << YAML::Value << pose.orientation.z;
	out << YAML::Key << "w" << YAML::Value << pose.orientation.w;
	out << YAML::EndMap << YAML::EndMap;

	return out;
}

namespace YAML {
	template<>
	struct convert<geometry_msgs::Pose> {
		static Node encode(const geometry_msgs::Pose& pose)
			{
				Node root;
				Node position_node;
				Node orientation_node;
				std::stringstream ss;

				ss << pose.position.x;
				position_node["x"] = ss.str();
				ss.str(std::string());
				ss << pose.position.y;
				position_node["y"] = ss.str();
				ss.str(std::string());
				ss << pose.position.z;
				position_node["z"] = ss.str();
				ss.str(std::string());

				ss << pose.orientation.x;
				orientation_node["x"] = ss.str();
				ss.str(std::string());
				ss << pose.orientation.y;
				orientation_node["y"] = ss.str();
				ss.str(std::string());
				ss << pose.orientation.z;
				orientation_node["z"] = ss.str();
				ss.str(std::string());
				ss << pose.orientation.w;
				orientation_node["w"] = ss.str();
				ss.str(std::string());

				root["position"] = position_node;
				root["orientation"] = orientation_node;

				return root;
			}

		static bool decode(const Node& node, geometry_msgs::Pose& pose)
			{
				pose.position.x = node["position"]["x"].as<double>();
				pose.position.y = node["position"]["y"].as<double>();
				pose.position.z = node["position"]["z"].as<double>();
				pose.orientation.x = node["orientation"]["x"].as<double>();
				pose.orientation.y = node["orientation"]["y"].as<double>();
				pose.orientation.z = node["orientation"]["z"].as<double>();
				pose.orientation.w = node["orientation"]["w"].as<double>();

				return true;
			}
	};
}

YAML::Emitter& operator<<(YAML::Emitter& out, const footstep_plan::FootStepPlan& plan_msg)
{
	out << YAML::BeginSeq;
	for(footstep_plan::FootStepPlan::_steps_type::const_iterator step = plan_msg.steps.begin();
		step != plan_msg.steps.end();
		step++)
	{
		out << YAML::BeginMap;
		out << YAML::Key << "step_index" << YAML::Value << step->step_index;
		out << YAML::Key << "foot_index" << YAML::Value << step->foot_index;
		out << YAML::Key << "duration" << YAML::Value << step->duration;
		out << YAML::Key << "pose" << YAML::Value << step->pose;
		out << YAML::Key << "swing_height" << YAML::Value << step->swing_height;
		out << YAML::EndMap;
	}
		
	out << YAML::EndSeq;
	return out;
}

YAML::Emitter& operator<<(YAML::Emitter& out, footstep_plan::FootStepPlan::ConstPtr& plan_msg)
{
	out << *plan_msg;
	return out;
}

YAML::Emitter& operator<<(YAML::Emitter& out, footstep_plan::FootStepPlan::Ptr& plan_msg)
{
	out << *plan_msg;
	return out;
}

namespace YAML {
	template<>
	struct convert<footstep_plan::FootStepPlan> {
		static Node encode(const footstep_plan::FootStepPlan& rhs) {
			Node node;
			for(footstep_plan::FootStepPlan::_steps_type::const_iterator step = rhs.steps.begin();
				step != rhs.steps.end();
				step++)
			{
				Node step_node;
				std::stringstream ss;
				ss << step->step_index;
				step_node["step_index"] = ss.str();
				ss.str(std::string());
				ss << step->foot_index;
				step_node["foot_index"] = ss.str();
				ss.str(std::string());
				ss << step->duration;
				step_node["duration"] = ss.str();
				ss.str(std::string());
				ss << step->swing_height;
				step_node["swing_height"] = ss.str();
				ss.str(std::string());

				step_node["pose"] = convert<geometry_msgs::Pose>::encode(step->pose);

				node.push_back(step_node);
			}
		}

		static bool decode(const Node& node, footstep_plan::FootStepPlan& rhs) {
			if(!node.IsSequence())
				return false;

			rhs.steps.clear();
			for(int i=0; i<node.size(); i++)
			{
				atlas_msgs::AtlasBehaviorStepData step;
				step.step_index = node[i]["step_index"].as<int>();
				step.foot_index = node[i]["foot_index"].as<int>();
				step.duration = node[i]["duration"].as<double>();
				step.swing_height = node[i]["swing_height"].as<double>();
				step.pose.position.x = node[i]["pose"]["position"]["x"].as<double>();
				step.pose.position.y = node[i]["pose"]["position"]["y"].as<double>();
				step.pose.position.z = node[i]["pose"]["position"]["z"].as<double>();
				step.pose.orientation.x = node[i]["pose"]["orientation"]["x"].as<double>();
				step.pose.orientation.y = node[i]["pose"]["orientation"]["y"].as<double>();
				step.pose.orientation.z = node[i]["pose"]["orientation"]["z"].as<double>();
				step.pose.orientation.w = node[i]["pose"]["orientation"]["w"].as<double>();

				rhs.steps.push_back(step);
			}

			return true;
		}
	};
}

YAML::Emitter& operator<<(YAML::Emitter& out, const footstep_plan::FootStepPose& footstep_pose)
{
	out << YAML::BeginMap;
	out << YAML::Key << "left_foot" << YAML::Value << footstep_pose.left_foot;
	out << YAML::Key << "right_foot" << YAML::Value << footstep_pose.right_foot;
	out << YAML::EndMap;
	
	return out;
}

YAML::Emitter& operator<<(YAML::Emitter& out, const footstep_plan::Roadmap& roadmap)
{
	out << YAML::BeginMap;
	out << YAML::Key << "nodes" << YAML::Value << YAML::BeginMap;
	for(footstep_plan::RoadmapNodes::const_iterator node = roadmap.nodes.begin();
		node != roadmap.nodes.end();
		node++)
	{
		out << YAML::Key << node->first << YAML::Value << node->second;
	}
	out << YAML::EndMap;
	out << YAML::Key << "edges" << YAML::Value << YAML::BeginMap;
	for(footstep_plan::RoadmapEdges::const_iterator outer_edge = roadmap.edges.begin();
		outer_edge != roadmap.edges.end();
		outer_edge++)
	{
		out << YAML::Key << outer_edge->first << YAML::Value << YAML::BeginMap;
		for(footstep_plan::RoadmapInnerEdges::const_iterator inner_edge = outer_edge->second.begin();
			inner_edge != outer_edge->second.end();
			inner_edge++)
		{
			out << YAML::Key << inner_edge->first << YAML::Value << inner_edge->second;
		}
		out << YAML::EndMap;
	}
	out << YAML::EndMap << YAML::EndMap;
	
	return out;
}

namespace YAML {

	template<>
	struct convert<footstep_plan::FootStepPose> {
		static Node encode(const footstep_plan::FootStepPose& footstep_pose)
			{
				Node root;
				root["left_foot"] = convert<geometry_msgs::Pose>::encode(footstep_pose.left_foot);
				root["right_foot"] = convert<geometry_msgs::Pose>::encode(footstep_pose.right_foot);
			}

		static bool decode(const Node& node, footstep_plan::FootStepPose& footstep_pose)
			{
				footstep_pose.left_foot = node["left_foot"].as<geometry_msgs::Pose>();
				footstep_pose.right_foot = node["right_foot"].as<geometry_msgs::Pose>();

				return true;
			}
	};
	
	template<>
	struct convert<footstep_plan::Roadmap> {
		static Node encode(const footstep_plan::Roadmap& roadmap)
		{
			Node root;
			Node nodes;
			Node outer_edges;
			std::stringstream ss;
			for(footstep_plan::RoadmapNodes::const_iterator node = roadmap.nodes.begin();
				node != roadmap.nodes.end();
				node++)
			{
				ss.str(std::string());
				ss << node->first;
				nodes[ss.str()] = convert<footstep_plan::FootStepPose>::encode(node->second);
			}
			root["nodes"] = nodes;

			for(footstep_plan::RoadmapEdges::const_iterator outer_edge = roadmap.edges.begin();
				outer_edge != roadmap.edges.end();
				outer_edge++)
			{
				Node inner_edges;
				for(footstep_plan::RoadmapInnerEdges::const_iterator inner_edge = outer_edge->second.begin();
					inner_edge != outer_edge->second.end();
					inner_edge++)
				{
					ss.str(std::string());
					ss << inner_edge->first;
					inner_edges[ss.str()] = convert<footstep_plan::FootStepPlan>::encode(inner_edge->second);
				}
				ss.str(std::string());
				ss << outer_edge->first;
				outer_edges[ss.str()] = inner_edges;
			}

			root["edges"] = outer_edges;

			return root;
		}

		static bool decode(const Node& node, footstep_plan::Roadmap& roadmap)
			{
				roadmap.nodes = node["nodes"].as<footstep_plan::RoadmapNodes>();
				roadmap.edges = node["edges"].as<footstep_plan::RoadmapEdges>();

				return true;
			}
	};
}


#endif
