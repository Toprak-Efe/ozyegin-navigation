//General/ROS Includes
#include <ros/ros.h>
#include <ros/console.h>
#include <iostream>
#include <stdexcept>
#include <ros/callback_queue.h>

//Navigation Includes
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <tf/transform_listener.h>
#include <cmath>
#include <array>

//PC2 Processing Includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <sensor_msgs/PointCloud2.h>
#include "pointcloud_process/occupancy_pos.h"

//Services Includes
#include <std_srvs/Empty.h>
#include <std_srvs/SetBool.h>

class cmd_relay{

	private:
	//Node handles for different data streams.
	ros::NodeHandle cmdh;
	ros::NodeHandle clnt1;
	ros::NodeHandle clnt2;

	//cmdh stream publisher-subscriber init
	ros::Subscriber sub1; //path 
	ros::Subscriber sub2;	//goal
	ros::Publisher pub;	//cmd_vel

	//To_height service objects (Retrieve costmap service)
	ros::ServiceClient client1;
	ros::ServiceClient client2;
	ros::ServiceClient client3;
	std_srvs::Empty process_msg;
	
	//tf objects
	tf::TransformListener listener;

	public:
	cmd_relay(){
		pub = cmdh.advertise<geometry_msgs::Twist> ("cmd_vel", 1);
		sub1 = cmdh.subscribe ("/ozyegin/path", 1, &cmd_relay::pathCallback, this);
		sub2 = cmdh.subscribe ("/move_base_simple/goal", 1, &cmd_relay::goalCallback, this);

		client1 = cmdh.serviceClient<std_srvs::Empty>("/ozyegin/services/cloud_process");
		client2 = clnt1.serviceClient<pointcloud_process::occupancy_pos>("/ozyegin/services/pathfind");
		client3 = clnt2.serviceClient<std_srvs::SetBool>("ozyegin/services/check_path");

		snapshot();
	}

	//Location Retrieval
	geometry_msgs::PoseStamped rover_location(){
        geometry_msgs::PoseStamped origin_pose;

        origin_pose.pose.orientation.y = 1.0;
        origin_pose.header.frame_id = "base_link";

        geometry_msgs::PoseStamped rover_pose;

		listener.transformPose("map", origin_pose, rover_pose);

        return rover_pose;
    }

	//Navigation Section
	void goalCallback(const geometry_msgs::PoseStampedConstPtr& goal_msg){
		pointcloud_process::occupancy_pos service_msg;
		geometry_msgs::PoseStamped rover_position = rover_location();

		service_msg.request.pose_x = goal_msg->pose.position.x;
		service_msg.request.pose_y = goal_msg->pose.position.y;

		if (client2.call(service_msg)){
			ROS_INFO("Path Request Succesful.");
		}
		else{
			ROS_INFO("Path Request Failed.");
		}		
		return;
	}

	void pathCallback(const nav_msgs::PathConstPtr& path_msg){
		pointcloud_process::occupancy_pos service_msg;
		service_msg.request.pose_x = path_msg->poses.back().pose.position.x;
		service_msg.request.pose_y = path_msg->poses.back().pose.position.y;
		for (geometry_msgs::PoseStamped position : path_msg->poses){
			if (!(to_pose(position))){
				//request service from pathfinder
				if (client2.call(service_msg)){
					ROS_INFO("Path Request Sucessful.");
				}
				else{
					ROS_INFO("Path Request Failed.");
				}	
				break;
			}
		}
	}

	bool to_pose(geometry_msgs::PoseStamped goal_msg){
		//Set up theta, the angle between the object and our rover from the front.
		double theta;
		
		//Set up position_transformed, the global position of our goal.
		geometry_msgs::PoseStamped position_transformed;
		
		//Apply map -> base_link transformation to goal_msg initially.
		listener.transformPose("base_link", goal_msg, position_transformed);

		//Calculate initial distance from point.
		double distance = sqrt(pow(position_transformed.pose.position.x, 2.0) + pow(position_transformed.pose.position.y, 2.0));

		//Movement loop to reach the target.
		while (distance > 0.50){

			//If statements for different situtations where our y values might be differently signed. We want
			//the theta to be positive for every coordinate as it signifies the deviation from the positive x axis
			//regardless of direction.
			if (position_transformed.pose.position.y <= 0)
			{
				theta = atan(position_transformed.pose.position.x/position_transformed.pose.position.y) + (3.1421/2);
				cmd_relay::turn(theta, "left");
			}
			else{
				theta = -atan(position_transformed.pose.position.x/position_transformed.pose.position.y) + (3.1421/2);
				cmd_relay::turn(theta, "right");
			}

			//Forward: x
			//Backwards: -x
			//Right: -y
			//Left: y
			
			forward();

			//Restate position and recalculate distance.
			listener.transformPose("base_link", goal_msg, position_transformed);
			distance = sqrt(pow(position_transformed.pose.position.x, 2.0) + pow(position_transformed.pose.position.y, 2.0));
		}

		//Scan forward.
		snapshot();

		//Check for path validity via service call
		std_srvs::SetBool collision_msg;

		if (client3.call(collision_msg)){
			ROS_INFO("Check Succesful.");
			if (collision_msg.response.success == 1){
				ROS_INFO("Collision detected, terminating.");
				return false;
			}
		}
		else{
			ROS_INFO("Check Failed.");
		}

		return true;
		//
	}

	void turn(double angle, std::string direction)
	{
		//Error checks.
		if (!((direction == "right") || (direction == "left"))){
			throw std::invalid_argument("direction argument must either be 'left' or 'right'.");
		}

		//Create the Twist rotate_command for deployment.
		geometry_msgs::Twist rotate_command;
		
		//Change rotate_command Z sign depending on direction.
		if (direction == "right") {rotate_command.angular.z = +0.50;}
		else {rotate_command.angular.z = -0.50;}

		ROS_INFO("Initiating Turning.");
		//Fine-tuned rotation loop.
		ros::Rate rate(2.0);
		for (double i = 0.0; i <= (angle); i += 0.1){
			pub.publish(rotate_command);
			rate.sleep();
		}
		ROS_INFO("Turning Executed.");
	}

	void forward(){
		//Generate a Twist forward_command with the move forward message.
		geometry_msgs::Twist forward_command;
		forward_command.linear.x = 1.0;
		
		for (int i = 1; i < 10; i++){
		//Publish message.
		pub.publish(forward_command);

		//Wait a little.
		ros::Rate rate(5);
		rate.sleep();
		}

		//Debugging Message.
		ROS_INFO("Forward Executed.");
	}

	void snapshot(){
		if (client1.call(process_msg)){
			ROS_INFO("Process Succesful.");
		}
		else{
			ROS_INFO("Process Failed.");
		}
	}

	void scan(){
		//Turn around 360 degrees and take a snapshot for every 120 degrees.
		snapshot();
		for (float i = 0.0; i < 2.0; i++){
		turn(6.28/3.0, "right");
		snapshot();
		}
	}

	
};

int main(int argc, char **argv)
{	
	//Initialize ROS interface, Initialize object, wait for callbacks.
	ros::init (argc, argv, "goal_commander");
	cmd_relay commander;
	ros::spin();
}
