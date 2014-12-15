#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <footstep_plan/FootStepPlan.h>

#include <footstep_plan/yaml_conversions.h>
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
	for(footstep_plan::FootStepPlan::_steps_type::const_iterator step = plan.steps.begin();
		step != plan.steps.end();
		step++)
	{
		visualization_msgs::Marker footstep_marker;
		footstep_marker.header.frame_id = "/map";
		footstep_marker.header.stamp = ros::Time::now();
		footstep_marker.ns = "footsteps";
		footstep_marker.id = step->step_index;
		footstep_marker.type = visualization_msgs::Marker::CUBE;
		footstep_marker.action = visualization_msgs::Marker::ADD;
		footstep_marker.pose = step->pose;
		footstep_marker.scale.x = .25;
		footstep_marker.scale.y = .13;
		footstep_marker.scale.z = .05;

		footstep_marker.color.a = .8;
		if(step->foot_index == 0)
		{
			footstep_marker.color.r = 1.0;
			footstep_marker.color.g = 0.0;
			footstep_marker.color.b = 0.0;
		}
		else
		{
			footstep_marker.color.r = 0.0;
			footstep_marker.color.g = 1.0;
			footstep_marker.color.b = 0.0;
		}

		footstep_markers.markers.push_back(footstep_marker);
	}

	ros::Rate loop_rate(1);
	while(ros::ok())
	{
		marker_pub.publish(footstep_markers);
		loop_rate.sleep();
	}
}
