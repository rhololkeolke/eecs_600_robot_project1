#include <ros/ros.h>
#include <footstep_plan/FootStepPlan.h>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <string>
#include <footstep_plan/yaml_conversions.h>

ros::Subscriber plan_sub;
footstep_plan::FootStepPlan::ConstPtr plan;

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
