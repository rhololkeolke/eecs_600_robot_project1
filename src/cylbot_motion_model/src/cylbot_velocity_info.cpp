#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <cstdio>

namespace gazebo
{
	class VelocityInfoPublisher : public ModelPlugin
	{
	public:
		void Load(physics::ModelPtr _parent, sdf::ElementPtr elem)
			{
				if(!ros::isInitialized())
				{
					printf("ROS is NOT intialized. Not using velocity info plugin\n");
					return;
				}

				this->model = _parent;

				vel_info_pub = nh.advertise<geometry_msgs::TwistStamped>("/cylbot/velocity" , 1);

				twist_msg.header.stamp = ros::Time::now();
				twist_msg.header.frame_id = "/base_link";
				
				vel_info_pub.publish(twist_msg);

				this->updateConnection = event::Events::ConnectWorldUpdateBegin(
					boost::bind(&VelocityInfoPublisher::OnUpdate, this, _1));
			}

		void OnUpdate(const common::UpdateInfo& info)
			{
				math::Vector3 linear_vel = this->model->GetRelativeLinearVel();
				math::Vector3 angular_vel = this->model->GetRelativeAngularVel();

				twist_msg.header.stamp = ros::Time::now();
				
				// get translational velocity in plane
				twist_msg.twist.linear.x = linear_vel.x;
				twist_msg.twist.linear.y = linear_vel.y;

				// get the yaw
				twist_msg.twist.angular.z = angular_vel.z;

				vel_info_pub.publish(twist_msg);
			}

	private:
		physics::ModelPtr model;
		event::ConnectionPtr updateConnection;

		ros::NodeHandle nh;
		ros::Publisher vel_info_pub;

		geometry_msgs::TwistStamped twist_msg;
	};

	GZ_REGISTER_MODEL_PLUGIN(VelocityInfoPublisher)
}
