#include <footstep_plan/FootStepPlan.h>
#include <yaml-cpp/yaml.h>
#include <sstream>
#include <string>

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
		out << YAML::Key << "pose" << YAML::Value << YAML::BeginMap;
		out << YAML::Key << "position" << YAML::Value << YAML::BeginMap;
		out << YAML::Key << "x" << YAML::Value << step->pose.position.x;
		out << YAML::Key << "y" << YAML::Value << step->pose.position.y;
		out << YAML::Key << "z" << YAML::Value << step->pose.position.z;
		out << YAML::EndMap;
		out << YAML::Key << "orientation" << YAML::Value << YAML::BeginMap;
		out << YAML::Key << "x" << YAML::Value << step->pose.orientation.x;
		out << YAML::Key << "y" << YAML::Value << step->pose.orientation.y;
		out << YAML::Key << "z" << YAML::Value << step->pose.orientation.z;
		out << YAML::Key << "w" << YAML::Value << step->pose.orientation.w;
		out << YAML::EndMap << YAML::EndMap;
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

				Node pose_node;
				Node position_node;
				ss << step->pose.position.x;
				position_node["x"] = ss.str();
				ss.str(std::string());
				ss << step->pose.position.y;
				position_node["y"] = ss.str();
				ss.str(std::string());
				ss << step->pose.position.z;
				position_node["z"] = ss.str();
				ss.str(std::string());
				Node orientation_node;
				ss << step->pose.orientation.x;
				orientation_node["x"] = ss.str();
				ss.str(std::string());
				ss << step->pose.orientation.y;
				orientation_node["y"] = ss.str();
				ss.str(std::string());
				ss << step->pose.orientation.z;
				orientation_node["z"] = ss.str();
				ss.str(std::string());
				ss << step->pose.orientation.w;
				orientation_node["w"] = ss.str();
				ss.str(std::string());

				pose_node["position"] = position_node;
				pose_node["orientation_node"] = orientation_node;
				step_node["pose"] = pose_node;

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
