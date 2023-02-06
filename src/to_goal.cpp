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
#include <tf/transform_listener.h>
#include <cmath>

//PC2 Processing Includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <sensor_msgs/PointCloud2.h>

//Costmap Services Includes
#include <std_srvs/Empty.h>

class cmd_relay{

	private:
	//Node handles for different data streams.
	ros::NodeHandle cmdh;

	//cmdh stream publisher-subscriber init
	ros::Subscriber sub;
	ros::Publisher pub;
	

	//To_height service objects (Retrieve costmap service)
	ros::ServiceClient client;
	std_srvs::Empty process_msg;
	
	//tf objects
	tf::TransformListener listener;
	tf::StampedTransform transform;

	public:
	cmd_relay(){
		pub = cmdh.advertise<geometry_msgs::Twist> ("cmd_vel", 1);

		sub = cmdh.subscribe ("/move_base_simple/goal", 1, &cmd_relay::posesCallback, this);

		client = cmdh.serviceClient<std_srvs::Empty>("/ozyegin/services/cloud_process");

		snapshot();
	}

	//Navigation Section

	void posesCallback(const geometry_msgs::PoseStampedConstPtr& goal_msg){
		//Set up theta, the angle between the object and our rover from the front.
		double theta;
		
		//Set up position_transformed, the global position of our goal.
		geometry_msgs::PoseStamped position_transformed;
		
		//Apply map -> base_footprint transformation to goal_msg initially.
		listener.transformPose("base_footprint", *goal_msg, position_transformed);

		//Calculate initial distance from point.
		double distance = sqrt(pow(position_transformed.pose.position.x, 2.0) + pow(position_transformed.pose.position.y, 2.0));

		//Movement loop to reach the target.
		while (distance > 0.25){
			ROS_INFO("Beginning Movement Cycle.");
			

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

			//For debugging.
			std::cout << "X: " << std::to_string(position_transformed.pose.position.x) << std::endl;
			std::cout << "Y: " << std::to_string(position_transformed.pose.position.y) << std::endl;
			std::cout << "0: " << std::to_string(theta) << std::endl;
			
			forward();

			//Restate position and recalculate distance.
			listener.transformPose("base_footprint", *goal_msg, position_transformed);
			distance = sqrt(pow(position_transformed.pose.position.x, 2.0) + pow(position_transformed.pose.position.y, 2.0));
		}

		//Scan around.
		scan();
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
		if (client.call(process_msg)){
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
