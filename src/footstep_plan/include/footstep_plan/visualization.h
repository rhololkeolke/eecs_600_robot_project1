#ifndef FOOTSTEP_PLAN__VISUALIZATION_H_
#define FOOTSTEP_PLAN__VISUALIZATION_H_

#include <visualization_msgs/MarkerArray.h>
#include <footstep_plan/FootStepPlan.h>
#include <footstep_plan/FootStepPose.h>
#include <footstep_plan/roadmap.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Point.h>
#include <string>


namespace footstep_plan {
	geometry_msgs::Point convertStepToMarker(const atlas_msgs::AtlasBehaviorStepData& step,
											 visualization_msgs::MarkerArray* markers,
											 unsigned int id_offset=0,
											 std::string frame_id="/map")
	{
		visualization_msgs::Marker footstep_marker;
		footstep_marker.header.frame_id = frame_id;
		footstep_marker.header.stamp = ros::Time::now();
		footstep_marker.ns = "footsteps";
		footstep_marker.id = step.step_index + id_offset;
		footstep_marker.type = visualization_msgs::Marker::CUBE;
		footstep_marker.action = visualization_msgs::Marker::ADD;
		footstep_marker.pose = step.pose;
		footstep_marker.scale.x = .25;
		footstep_marker.scale.y = .13;
		footstep_marker.scale.z = .05;


		// calculate the path line point
		tf::Transform path_transform;
		path_transform.setOrigin(tf::Vector3(step.pose.position.x,
											 step.pose.position.y,
											 step.pose.position.z));
		path_transform.setRotation(tf::Quaternion(step.pose.orientation.x,
												  step.pose.orientation.y,
												  step.pose.orientation.z,
												  step.pose.orientation.w));
		tf::Vector3 path_point;
	
		footstep_marker.color.a = .8;
		if(step.foot_index == 0)
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

		markers->markers.push_back(footstep_marker);
	
		geometry_msgs::Point geom_path_point;
		geom_path_point.x = path_point.getX();
		geom_path_point.y = path_point.getY();
		geom_path_point.z = path_point.getZ();

		return geom_path_point;
	}

	void convertPlanToMarkers(const footstep_plan::FootStepPlan& plan,
							  visualization_msgs::MarkerArray* markers,
							  std::string frame_id="/map")
	{

		int id_offset = markers->markers.size();
	
		visualization_msgs::Marker footstep_path;
		footstep_path.header.frame_id = frame_id;
		footstep_path.header.stamp = ros::Time::now();
		footstep_path.ns = "footstep_path";
		footstep_path.id = id_offset;
		footstep_path.type = visualization_msgs::Marker::LINE_STRIP;
		footstep_path.action = visualization_msgs::Marker::ADD;
		footstep_path.scale.x = .05;
		footstep_path.pose.orientation.w = 1.0;
		footstep_path.color.a = .8;
		footstep_path.color.r = 0.0;
		footstep_path.color.g = 0.0;
		footstep_path.color.b = 1.0;

		for(footstep_plan::FootStepPlan::_steps_type::const_iterator step = plan.steps.begin();
			step != plan.steps.end();
			step++)
		{
			geometry_msgs::Point path_point = convertStepToMarker(*step,
																  markers,
																  id_offset,
																  frame_id);
			footstep_path.points.push_back(path_point);
			footstep_path.colors.push_back(footstep_path.color);
		}

		markers->markers.push_back(footstep_path);
	}

	void convertPlanToMarkers(const footstep_plan::FootStepPlan& plan,
							  const footstep_plan::FootStepPose& start_pose,
							  const footstep_plan::FootStepPose& end_pose,
							  visualization_msgs::MarkerArray* markers,
							  std::string frame_id="/map")
	{
		int id_offset = markers->markers.size();

		visualization_msgs::Marker footstep_path;
		footstep_path.header.frame_id = frame_id;
		footstep_path.header.stamp = ros::Time::now();
		footstep_path.ns = "footstep_path";
		footstep_path.id = id_offset;
		footstep_path.type = visualization_msgs::Marker::LINE_STRIP;
		footstep_path.action = visualization_msgs::Marker::ADD;
		footstep_path.scale.x = .05;
		footstep_path.pose.orientation.w = 1.0;
		footstep_path.color.a = .8;
		footstep_path.color.r = 0.0;
		footstep_path.color.g = 0.0;
		footstep_path.color.b = 1.0;

		// calculate the path line point
		tf::Transform path_transform;
		path_transform.setOrigin(tf::Vector3(start_pose.left_foot.position.x,
											 start_pose.left_foot.position.y,
											 start_pose.left_foot.position.z));
		path_transform.setRotation(tf::Quaternion(start_pose.left_foot.orientation.x,
												  start_pose.left_foot.orientation.y,
												  start_pose.left_foot.orientation.z,
												  start_pose.left_foot.orientation.w));
		tf::Vector3 path_point = path_transform(tf::Vector3(0,-.15, 0));
		geometry_msgs::Point geom_path_point;
		geom_path_point.x = path_point.getX();
		geom_path_point.y = path_point.getY();
		geom_path_point.z = path_point.getZ();
		footstep_path.points.push_back(geom_path_point);
		footstep_path.colors.push_back(footstep_path.color);
		
		for(footstep_plan::FootStepPlan::_steps_type::const_iterator step = plan.steps.begin();
			step != plan.steps.end();
			step++)
		{
			geometry_msgs::Point path_point = convertStepToMarker(*step,
																  markers,
																  id_offset,
																  frame_id);
			footstep_path.points.push_back(path_point);
			footstep_path.colors.push_back(footstep_path.color);
		}

		// calculate the path line point
		path_transform.setOrigin(tf::Vector3(end_pose.left_foot.position.x,
											 end_pose.left_foot.position.y,
											 end_pose.left_foot.position.z));
		path_transform.setRotation(tf::Quaternion(end_pose.left_foot.orientation.x,
												  end_pose.left_foot.orientation.y,
												  end_pose.left_foot.orientation.z,
												  end_pose.left_foot.orientation.w));
		path_point = path_transform(tf::Vector3(0,-.15, 0));
		geom_path_point.x = path_point.getX();
		geom_path_point.y = path_point.getY();
		geom_path_point.z = path_point.getZ();
		footstep_path.points.push_back(geom_path_point);
		footstep_path.colors.push_back(footstep_path.color);

		markers->markers.push_back(footstep_path);

	}

	void convertPoseToMarkers(const footstep_plan::FootStepPose& pose,
							  visualization_msgs::MarkerArray* markers,
							  std::string frame_id="/map")
	{
		int id = markers->markers.size();

		visualization_msgs::Marker pose_marker;
		pose_marker.header.frame_id = frame_id;
		pose_marker.header.stamp = ros::Time::now();
		pose_marker.ns = "footstep_pose_orientation";
		pose_marker.id = id;
		pose_marker.type = visualization_msgs::Marker::ARROW;
		pose_marker.action = visualization_msgs::Marker::ADD;
		pose_marker.scale.x = .24;
		pose_marker.scale.y = .02;
		pose_marker.scale.z = .02;
		pose_marker.color.a = .8;
		pose_marker.color.r = 1.0;
		pose_marker.color.g = 1.0;
		pose_marker.color.b = 0.0;
		pose_marker.pose.position.x = (pose.left_foot.position.x + pose.right_foot.position.x)/2.0;
		pose_marker.pose.position.y = (pose.left_foot.position.y + pose.right_foot.position.y)/2.0;
		pose_marker.pose.position.z = (pose.left_foot.position.z + pose.right_foot.position.z)/2.0;	

		double left_yaw = tf::getYaw(pose.left_foot.orientation);
		double right_yaw = tf::getYaw(pose.right_foot.orientation);

		pose_marker.pose.orientation = tf::createQuaternionMsgFromYaw((left_yaw + right_yaw)/2.0);

		visualization_msgs::Marker left_foot_marker;
		left_foot_marker.header.frame_id = frame_id;
		left_foot_marker.header.stamp = ros::Time::now();
		left_foot_marker.ns = "footstep_pose_feet";
		left_foot_marker.id = id;
		left_foot_marker.type = visualization_msgs::Marker::CUBE;
		left_foot_marker.action = visualization_msgs::Marker::ADD;
		left_foot_marker.pose = pose.left_foot;
		left_foot_marker.scale.x = .25;
		left_foot_marker.scale.y = .13;
		left_foot_marker.scale.z = .05;
		left_foot_marker.color.a = .8;
		left_foot_marker.color.r = 1.0;
		left_foot_marker.color.g = 0.0;
		left_foot_marker.color.b = 1.0;

		visualization_msgs::Marker right_foot_marker = left_foot_marker;
		right_foot_marker.id++;
		right_foot_marker.pose = pose.right_foot;

		markers->markers.push_back(pose_marker);
		markers->markers.push_back(left_foot_marker);
		markers->markers.push_back(right_foot_marker);
			
	}

	void convertRoadmapToMarkers(const footstep_plan::Roadmap& roadmap,
								 visualization_msgs::MarkerArray* markers,
								 std::string frame_id="/map")
	{
		for(footstep_plan::RoadmapNodes::const_iterator node = roadmap.nodes.begin();
			node != roadmap.nodes.end();
			node++)
		{
			convertPoseToMarkers(node->second, markers, frame_id);
		}

		for(footstep_plan::RoadmapEdges::const_iterator outer_edge = roadmap.edges.begin();
			outer_edge != roadmap.edges.end();
			outer_edge++)
		{
			for(footstep_plan::RoadmapInnerEdges::const_iterator inner_edge = outer_edge->second.begin();
				inner_edge != outer_edge->second.end();
				inner_edge++)
			{
				footstep_plan::RoadmapNodes::const_iterator start_node = roadmap.nodes.find(outer_edge->first);
				footstep_plan::RoadmapNodes::const_iterator end_node = roadmap.nodes.find(inner_edge->first);
				convertPlanToMarkers(inner_edge->second,
									 start_node->second,
									 end_node->second,
									 markers,
									 frame_id);
			}
		}
	}
}

#endif
