#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <footstep_plan/FootStepPlan.h>

#include <footstep_plan/yaml_conversions.h>
#include <yaml-cpp/yaml.h>
#include <string>

#include <tf/transform_datatypes.h>

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

	visualization_msgs::Marker footstep_path;
	footstep_path.header.frame_id = "/map";
	footstep_path.header.stamp = ros::Time::now();
	footstep_path.ns = "footstep_path";
	footstep_path.id = 0;
	footstep_path.type = visualization_msgs::Marker::LINE_STRIP;
	footstep_path.action = visualization_msgs::Marker::ADD;
	footstep_path.scale.x = .01;
	footstep_path.pose.orientation.w = 1.0;
	footstep_path.color.a = .8;
	footstep_path.color.r = 0.0;
	footstep_path.color.g = 0.0;
	footstep_path.color.b = 1.0;

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


		// calculate the path line point
		tf::Transform path_transform;
		path_transform.setOrigin(tf::Vector3(step->pose.position.x,
											 step->pose.position.y,
											 step->pose.position.z));
		path_transform.setRotation(tf::Quaternion(step->pose.orientation.x,
												   step->pose.orientation.y,
												   step->pose.orientation.z,
												   step->pose.orientation.w));
		tf::Vector3 path_point;
		
		footstep_marker.color.a = .8;
		if(step->foot_index == 0)
		{
			// set red
			footstep_marker.color.r = 1.0;
			footstep_marker.color.g = 0.0;
			footstep_marker.color.b = 0.0;

			path_point = path_transform(tf::Vector3(0,-.15, 0));
			
		}
		else
		{
			// set green
			footstep_marker.color.r = 0.0;
			footstep_marker.color.g = 1.0;
			footstep_marker.color.b = 0.0;

			path_point = path_transform(tf::Vector3(0, .15, 0));
		}

		geometry_msgs::Point geom_path_point;
		geom_path_point.x = path_point.getX();
		geom_path_point.y = path_point.getY();
		geom_path_point.z = path_point.getZ();
		footstep_path.points.push_back(geom_path_point);
		footstep_path.colors.push_back(footstep_path.color);

		footstep_markers.markers.push_back(footstep_marker);
	}

	footstep_markers.markers.push_back(footstep_path);

	ros::Rate loop_rate(1);
	while(ros::ok())
	{
		marker_pub.publish(footstep_markers);
		loop_rate.sleep();
	}
}
