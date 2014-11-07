#include <cylbot_mcl/pose_cloud.h>
#include <boost/random/normal_distribution.hpp>
#include <boost/random/uniform_real_distribution.hpp>
#include <boost/bind.hpp>
#include <cmath>
#include <vector>
#include <utility>
#include <algorithm>
#include <tf/transform_datatypes.h>

namespace cylbot_mcl
{
	inline int round(const double a) { return int (a + 0.5); }
	
	PoseCloud2D::PoseCloud2D(const RobotModel& model, const nav_msgs::OccupancyGrid& map)
	{
		this->pose_array.header.frame_id = "/map";

		this->model = model;
		
		this->map = map;

		num_sensor_updates = 0;
	}

	PoseCloud2D::PoseCloud2D(const RobotModel& model,
							 const geometry_msgs::PoseWithCovarianceStamped& initial_pose,
							 const int num_particles,
							 const nav_msgs::OccupancyGrid& map)
	{
		this->pose_array.header.frame_id = "/map";

		this->model = model;

		this->map = map;
		
		this->resetCloud(initial_pose, num_particles);

		num_sensor_updates = 0;
	}

	void PoseCloud2D::resetCloud(const geometry_msgs::PoseWithCovarianceStamped& initial_pose, const int num_particles)
	{
		Eigen::VectorXd mean(3);
		Eigen::MatrixXd covar(3,3);

		mean << initial_pose.pose.pose.position.x,
			    initial_pose.pose.pose.position.y,
			    tf::getYaw(initial_pose.pose.pose.orientation);
		covar << initial_pose.pose.covariance[0], 0,                                  0,
			     0,                               initial_pose.pose.covariance[7],    0,
			     0,                               0,                                  initial_pose.pose.covariance[35];

		Eigen::MatrixXd normTransform(3, 3);
		Eigen::LLT<Eigen::MatrixXd> cholSolver(covar);
		
		if(cholSolver.info() == Eigen::Success)
		{
			normTransform = cholSolver.matrixL();
		}
		else
		{
			Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigenSolver(covar);
			normTransform = eigenSolver.eigenvectors() * eigenSolver.eigenvalues().cwiseSqrt().asDiagonal();
		}
		
		Eigen::MatrixXd samples = (normTransform * Eigen::MatrixXd::NullaryExpr(3, num_particles, randn)).colwise() + mean;
		
		pose_array.poses.clear();
		for(int i=0; i<num_particles; i++)
		{
			geometry_msgs::Pose pose;
			pose.position.x = samples(0, i);
			pose.position.y = samples(1, i);
			pose.position.z = 0;

			// generate random yaw then convert it to a quaternion
			pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, samples(2, i));
		
			pose_array.poses.push_back(pose);
		}
	}

	void PoseCloud2D::motionUpdate(const geometry_msgs::Twist& u, double dt)
	{
		last_cmd = u;
		// don't do anything if the robot isn't moving
		if(fabs(u.linear.y) < 0.1 && fabs(u.angular.z) < 0.1)
			return;

		double v_hat_variance = model.alpha[0]*u.linear.y*u.linear.y + model.alpha[1]*u.angular.z*u.angular.z;
		double w_hat_variance = model.alpha[2]*u.linear.y*u.linear.y + model.alpha[3]*u.angular.z*u.angular.z;
		double gamma_variance = model.alpha[4]*u.linear.y*u.linear.y + model.alpha[5]*u.angular.z*u.angular.z;
		
		boost::normal_distribution<> v_hat_dist(0.0, sqrt(v_hat_variance));
		boost::normal_distribution<> w_hat_dist(0.0, sqrt(w_hat_variance));
		boost::normal_distribution<> gamma_dist(0.0, sqrt(gamma_variance));
		
		double v_hat = u.linear.y + v_hat_dist(rng);
		double w_hat = u.angular.z + w_hat_dist(rng);
		double gamma = gamma_dist(rng);
		
		double v_w_ratio = v_hat/w_hat;

		for(geometry_msgs::PoseArray::_poses_type::iterator pose = pose_array.poses.begin();
			pose != pose_array.poses.end();
			pose++)
		{
			double yaw = tf::getYaw(pose->orientation);

			double x_prime = pose->position.x - v_w_ratio*sin(yaw) + v_w_ratio*sin(yaw + w_hat*dt);
			double y_prime = pose->position.y + v_w_ratio*cos(yaw) - v_w_ratio*cos(yaw + w_hat*dt);
			double yaw_prime = yaw + w_hat*dt + gamma*dt;

			pose->position.x = x_prime;
			pose->position.y = y_prime;
			pose->orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, yaw_prime);
		}

	}

	void PoseCloud2D::sensorUpdate(const pcl::PointCloud<pcl::PointXYZ>& beam_ends,
								   const tf::Vector3& beam_start)
	{
		// this type is used as a temporary datastructure in this function only
		// this typedef just saves some typing and makes the later code more readable
		typedef geometry_msgs::PoseArray::_poses_type::iterator WeightPairFirst;
		typedef double WeightPairSecond;
		typedef std::pair<WeightPairFirst, WeightPairSecond> WeightPair;
		typedef std::vector<WeightPair> Weights;
		
		// if we haven't set a map yet then don't do anything
		if(map.data.size() == 0)
			return;
		
		// don't perform sensor updates unless moving
		// this is to prevent the variance from trending to 0 with probability 1
		// see page 108 Resampling section of Probabilistic Robotics Textbook
		if(fabs(last_cmd.linear.y) < 0.1 && fabs(last_cmd.linear.z) < 0.1)
			return;

		//if(num_sensor_updates++ % 100 != 0)
		//	return;

		num_sensor_updates = 0;

		// store an iterator to a pose along with its computed weight
		Weights pose_weights;
		double total_weight = 0;
		double max_weight = 0;

		// for(pcl::PointCloud<pcl::PointXYZ>::const_iterator beam_end = beam_ends.points.begin();
		// 	beam_end != beam_ends.points.end();
		// 	beam_end++)
		// {
		// 	if(beam_end->x != beam_end->x || beam_end->y != beam_end->y || beam_end->z != beam_end->z)
		// 		ROS_WARN("sensorUpdate: beam_end has NaN value");
		// }

		// calculate the weight of each pose particle
		for(geometry_msgs::PoseArray::_poses_type::iterator pose = pose_array.poses.begin();
			pose != pose_array.poses.end();
			pose++)
		{
			double prob = getMeasurementProbability(*pose, beam_ends, beam_start);
			pose_weights.push_back(std::make_pair(pose, prob));
			total_weight += prob;

			if(prob > max_weight)
				max_weight = prob;
		}

		ROS_INFO_STREAM("total weight: " << total_weight);
		ROS_INFO_STREAM("Average prob: " << total_weight/pose_array.poses.size());
		ROS_INFO_STREAM("Max prob: " << max_weight);

		// normalize
		for(Weights::iterator weight_pair = pose_weights.begin();
			weight_pair != pose_weights.end();
			weight_pair++)
		{
			weight_pair->second /= total_weight;
			//ROS_INFO_STREAM(weight_pair->second);
		}

		double M = (double)(pose_weights.size());

		// create prob generator
		boost::random::uniform_real_distribution<> dist(0.0, 1.0/M);
		boost::variate_generator<boost::mt19937, boost::random::uniform_real_distribution<> > rand(rng, dist);

		// resample using low variance sampling from table 4.4 on page 110 of the Probabilistic robotics book
		geometry_msgs::PoseArray::_poses_type poses;

		double r = rand();
		double c = pose_weights.begin()->second;
		int i=0;
		for(int m=0; m<M; m++)
		{
			double U = r + m/M;
			while(U > c)
			{
				i++;
				c = c + pose_weights[i].second;
			}
			poses.push_back(*(pose_weights[i].first));
		}

		// update the main datastructure array
		pose_array.poses = poses;

	}

	void PoseCloud2D::mapUpdate(const nav_msgs::OccupancyGrid& map)
	{
		this->map = map;
	}

	geometry_msgs::PoseArray PoseCloud2D::getPoses()
	{
		return this->pose_array;
	}

	double PoseCloud2D::getMeasurementProbability(const geometry_msgs::Pose& pose,
												  const pcl::PointCloud<pcl::PointXYZ>& beam_ends,
												  const tf::Vector3 beam_start)
	{

		const double occ_threshold = .5;
		double probability = 1.0;
		double total_probability = 0;

		int x_origin = (int)fabs(map.info.origin.position.x/map.info.resolution);
		int y_origin = (int)fabs(map.info.origin.position.y/map.info.resolution);

		//ROS_INFO("x_origin: %d y_origin: %d width: %d height: %d", x_origin, y_origin, map.info.width, map.info.height);

		// construct the transform from the map to the pose
		tf::Transform pose_transform;
		pose_transform.setOrigin(tf::Vector3(pose.position.x, pose.position.y, pose.position.z));
		tf::Quaternion pose_rotation;
		tf::quaternionMsgToTF(pose.orientation, pose_rotation);
		pose_transform.setRotation(pose_rotation);

		// transform the beam_start to the estimated map frame
		tf::Vector3 map_beam_start = pose_transform(beam_start);
		map_beam_start.setZ(0);

		//ROS_INFO("map_beam_start: (%3.3f, %3.3f, %3.3f)", map_beam_start.getX(), map_beam_start.getY(), map_beam_start.getZ());

		// for(int i=0; i<map.info.width; i++)
		// {
		// 	for(int j=0; j<map.info.height; j++)
		// 	{
		// 		int occ = getCellOccupancy(i,j);
		// 		if(occ != 0)
		// 		{
		// 			ROS_INFO("(%d, %d) has occupancy: %d", i, j, occ);
		// 		}
		// 	}
		// }

		int num_points = 0;
		for(pcl::PointCloud<pcl::PointXYZ>::const_iterator beam_end = beam_ends.begin();
			beam_end != beam_ends.end();
			beam_end++)
		{
			if(num_points++ > 1)
				break;
			// transform the point to the estimated map frame
			tf::Vector3 map_beam_end = pose_transform(tf::Vector3(beam_end->x, beam_end->y, beam_end->z));
			map_beam_end.setZ(0);

			int end_coord_x = (int)(map_beam_end.getX()/map.info.resolution) + x_origin;
			int end_coord_y = (int)(map_beam_end.getY()/map.info.resolution) + y_origin;
			int endocc = getCellOccupancy(end_coord_x, end_coord_y);

			double beam_length = map_beam_end.distance(map_beam_start);

			//ROS_INFO("beam_length: %3.3f", beam_length);
			if(beam_length != beam_length)
			{
				ROS_INFO_STREAM("beam_end: " << *beam_end);
				ROS_INFO_STREAM("beam_start: " << beam_start);
				ROS_INFO_STREAM("map_beam_end: " << map_beam_end);
				ROS_INFO_STREAM("map_beam_start: " << map_beam_start);
			}

			tf::Vector3 map_beam_dir = (map_beam_end - map_beam_start).normalize();
			map_beam_dir *= 30.0;

			tf::Vector3 max_end = map_beam_dir + map_beam_start;
				
			int dx = (int)((max_end.getX() - map_beam_start.getX())/map.info.resolution);
			int dy = (int)((max_end.getY() - map_beam_start.getY())/map.info.resolution);
			int steps;

			double xIncrement, yIncrement;
			double x = map_beam_start.getX()/map.info.resolution;
			double y = map_beam_start.getY()/map.info.resolution;

			if(fabs(dx) > fabs(dy))
				steps = fabs(dx);
			else
				steps = fabs(dy);

			xIncrement = ((double)dx)/((double)steps);
			yIncrement = ((double)dy)/((double)steps);


			int occ = getCellOccupancy(round(x) + x_origin, round(y) + y_origin);
			//ROS_INFO("occ: %d", occ);
			if(occ > occ_threshold)
			{
//				ROS_INFO("Hit occupied cell");
				double end_x = x*map.info.resolution;
				double end_y = y*map.info.resolution;

				double x_diff = end_x - map_beam_start.getX();
				double y_diff = end_y - map_beam_start.getY();
				
				double map_dist = sqrt(x_diff*x_diff + y_diff*y_diff);

				if(beam_length != beam_length)
					ROS_INFO("beam_length is NaN!");
				if(map_dist != map_dist)
					ROS_INFO("map_dist is NaN!");

				double prob = model.sensor_params.measurementProbability(beam_length, map_dist)/100.0;
				// if(prob < .001)
				// {
				// 	ROS_INFO("x: %3.3f y: %3.3f beam length: %3.3f, map_dist: %3.3f, prob: %f", end_x, end_y, beam_length, map_dist, prob);
				// }
				total_probability += prob;
				probability *= prob;
				continue;
			}
			bool hitOccupied = false;
			for(int k=0; k<steps-1; k++)
			{
				x += xIncrement;
				y += yIncrement;

				int occ = getCellOccupancy(round(x)+x_origin, round(y)+y_origin);
//				ROS_INFO("occ: %d", occ);
				if(occ > occ_threshold)
				{
					//ROS_INFO("Hit occupied cell");
					double end_x = x*map.info.resolution;
					double end_y = y*map.info.resolution;
					double x_diff = end_x - map_beam_start.getX();
					double y_diff = end_y - map_beam_start.getY();

					double map_dist = sqrt(x_diff*x_diff + y_diff*y_diff);

					if(beam_length != beam_length)
						ROS_INFO("beam_length is NaN!");
					if(map_dist != map_dist)
						ROS_INFO("map_dist is NaN!");

					double prob = model.sensor_params.measurementProbability(beam_length, map_dist)/100.0;
					// if(prob < .001)
					// {
					// 	ROS_INFO("x: %3.3f y: %3.3f beam length: %3.3f, map_dist: %3.3f, prob: %f", end_x, end_y, beam_length, map_dist, prob);
					// }
					total_probability += prob;
					probability *= prob;

					hitOccupied = true;
					break;
				}
			}

			// if no cell along the ray was hit then the distance should be the max sensor reading
			if(!hitOccupied)
			{
				double prob = model.sensor_params.measurementProbability(beam_length, 30.0)/100.0;
				// if(prob < .001)
				// {
				// 	ROS_INFO("beam length: %3.3f, map_dist: %3.3f, prob: %f", beam_length, 30.0, prob);
				// }
				total_probability += prob;
				probability *= prob;

			}
		}

		//ROS_INFO_STREAM("average probability:" << total_probability/beam_ends.points.size());

		//ROS_INFO_STREAM("probability:" << probability);
		return probability;
	}

	int PoseCloud2D::getCellOccupancy(const int x, const int y)
	{
		if(x < map.info.width && x >= 0 &&
		   y < map.info.height && y >= 0)
		{
			return map.data[y*map.info.width + x];
		}

		return -1;
	}
	
}
