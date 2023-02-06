#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <cstdio>

void odoCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
	static tf2_ros::StaticTransformBroadcaster broadcaster;
	tf2::Quaternion quat_tf(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
	quat_tf.normalize();

	geometry_msgs::Quaternion quat_msg = tf2::toMsg(quat_tf);	
        
	geometry_msgs::TransformStamped transformStamped;
	
        transformStamped.header.stamp = ros::Time::now();
        transformStamped.header.frame_id = "map";
        transformStamped.child_frame_id = "base_footprint";

        transformStamped.transform.translation.x = msg->pose.pose.position.x;
        transformStamped.transform.translation.y = msg->pose.pose.position.y;
        transformStamped.transform.translation.z = msg->pose.pose.position.z;
        transformStamped.transform.rotation = quat_msg;

        broadcaster.sendTransform(transformStamped);
}

int main(int argc, char** argv)
{	
	ros::init(argc, argv, "ozyegin_localizer");
	ros::NodeHandle sgb;

	ros::Subscriber sub = sgb.subscribe("/ground_truth", 1000, odoCallback);
	
	ros::spin();
}
