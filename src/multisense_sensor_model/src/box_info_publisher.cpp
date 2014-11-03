#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <cstdio>

namespace gazebo
{
	class BoxInfoPublisher : public ModelPlugin
	{
	public:
		void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/) 
		{
			if(!ros::isInitialized())
			{
				printf("ROS is NOT initialized. Not using plugin\n");
				return;
			}

			//box_info_pub = nh.advertise<visualization_msgs::Marker>("/gazebo/

			this->model = _parent;

			marker_msg.header.frame_id = "/map";
			marker_msg.header.stamp = ros::Time::now();
			marker_msg.ns = "gazebo_box_info";
			marker_msg.id = 0;
			marker_msg.type = visualization_msgs::Marker::CUBE;
			marker_msg.action = visualization_msgs::Marker::ADD;

			math::Pose world_pose = this->model->GetWorldPose();
			marker_msg.pose.position.x = world_pose.pos.x;
			marker_msg.pose.position.y = world_pose.pos.y;
			marker_msg.pose.position.z = world_pose.pos.z;
			marker_msg.pose.orientation.x = world_pose.rot.x;
			marker_msg.pose.orientation.y = world_pose.rot.y;
			marker_msg.pose.orientation.z = world_pose.rot.z;
			marker_msg.pose.orientation.w = world_pose.rot.w;

			physics::Link_V links = this->model->GetLinks();
			for(physics::Link_V::const_iterator link = links.begin();
				link != links.end();
				link++)
			{

				physics::Collision_V collisions = (*link)->GetCollisions();
				for(physics::Collision_V::const_iterator collision = collisions.begin();
					collision != collisions.end();
					collision++)
				{
					physics::ShapePtr shape((*collision)->GetShape());

					if(shape->HasType(physics::Base::BOX_SHAPE))
					{
						physics::BoxShape *box = static_cast<physics::BoxShape*>(shape.get());
						math::Vector3 size = box->GetSize();

						marker_msg.scale.x = size.x;
						marker_msg.scale.y = size.y;
						marker_msg.scale.z = size.z;
					}
				}
			}

			marker_msg.color.a = 0.5;
			marker_msg.color.g = 1.0;
			
			// create a latching publisher
			box_info_pub = nh.advertise<visualization_msgs::Marker>("/gazebo/" + this->model->GetName() + "/info", 1, true);

			box_info_pub.publish(marker_msg);

			this->updateConnection = event::Events::ConnectWorldUpdateBegin(
				boost::bind(&BoxInfoPublisher::OnUpdate, this, _1));
		}

		void OnUpdate(const common::UpdateInfo& /*_info*/)
		{
			// publish data here
			marker_msg.header.stamp = ros::Time::now();
			box_info_pub.publish(marker_msg);
		}

	private:
		physics::ModelPtr model;

		event::ConnectionPtr updateConnection;

		ros::NodeHandle nh;
		ros::Publisher box_info_pub;

		visualization_msgs::Marker marker_msg;
				
	};

		GZ_REGISTER_MODEL_PLUGIN(BoxInfoPublisher)
}
