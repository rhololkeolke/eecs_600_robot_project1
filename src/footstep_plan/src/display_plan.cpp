#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <footstep_plan/FootStepPlan.h>
#include <footstep_plan/yaml_conversions.h>
#include <footstep_plan/visualization.h>
#include <yaml-cpp/yaml.h>
#include <string>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "display_footstep_plan");

	ros::NodeHandle nh;

	ros::Publisher marker_pub = nh.advertise<visualization_msgs::MarkerArray>("footstep_plan", 1, true);

	ros::NodeHandle priv_nh("~");
	std::string input_file = "";
	priv_nh.getParam("input_file", input_file);

	ROS_ASSERT_MSG(!input_file.empty(), "Must specify an input file location");

	YAML::Node yaml_file = YAML::LoadFile(input_file.c_str());
	footstep_plan::FootStepPlan plan = yaml_file.as<footstep_plan::FootStepPlan>();

	ROS_DEBUG_STREAM(plan);

	visualization_msgs::MarkerArray footstep_markers;
	footstep_plan::convertPlanToMarkers(plan, &footstep_markers);
	
	ros::Rate loop_rate(1);
	while(ros::ok())
	{
		marker_pub.publish(footstep_markers);
		loop_rate.sleep();
	}
}
