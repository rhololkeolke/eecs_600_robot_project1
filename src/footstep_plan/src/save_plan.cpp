#include <ros/ros.h>
#include <footstep_plan/FootStepPlan.h>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <string>

ros::Subscriber plan_sub;
footstep_plan::FootStepPlan::ConstPtr plan;

YAML::Emitter& operator<<(YAML::Emitter& out, footstep_plan::FootStepPlan::ConstPtr& plan_msg)
{
	out << YAML::BeginSeq;
	for(footstep_plan::FootStepPlan::_steps_type::const_iterator step = plan_msg->steps.begin();
		step != plan_msg->steps.end();
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

void footStepPlanCallback(const footstep_plan::FootStepPlan::ConstPtr& plan_msg)
{
	plan = plan_msg;
	plan_sub.shutdown();
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "save_footstep_plan");

	ros::NodeHandle nh;

	ros::NodeHandle priv_nh("~");
	std::string output_file = "";
	priv_nh.getParam("output_file", output_file);

	ROS_ASSERT_MSG(!output_file.empty(), "Must specify an output file location");

	plan_sub = nh.subscribe<footstep_plan::FootStepPlan>("/footstep_plan", 1, &footStepPlanCallback);

	ros::Rate loop_rate(10);
	while(ros::ok() && plan == NULL)
	{
		ros::spinOnce();
		loop_rate.sleep();
	}

	YAML::Emitter emitter;
	emitter << plan;

	std::ofstream yaml_file(output_file.c_str());
	yaml_file << emitter.c_str();

	return 0;
}
