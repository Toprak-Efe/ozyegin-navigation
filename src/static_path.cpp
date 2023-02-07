#include <ros/ros.h>

#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_srvs/Empty.h>
#include <vector>

class static_publisher
{
    private:
    ros::NodeHandle nh;
    ros::Publisher pub;
    ros::ServiceServer server;

    public:

    static_publisher(){
        pub = nh.advertise<nav_msgs::Path> ("/ozyegin/path", 1, false);
        server = nh.advertiseService("/ozyegin/services/publish_path", &static_publisher::broadcast_path, this);
    }

    bool broadcast_path(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& res){
        nav_msgs::Path path;
        geometry_msgs::PoseStamped pose1;
        pose1.header.stamp = ros::Time::now();
        pose1.header.frame_id = "map";
        pose1.pose.position.x = 3.0;
        pose1.pose.orientation.z = 1.0;

        geometry_msgs::PoseStamped pose2;
        pose2.header.stamp = ros::Time::now();
        pose2.header.frame_id = "map";
        pose2.pose.position.x = 6.0;
        pose2.pose.orientation.z = 1.0;
    
        std::vector<geometry_msgs::PoseStamped> positions;
        positions.push_back(pose1);
        positions.push_back(pose2);

        path.header.frame_id = "map";
        path.header.stamp = ros::Time::now();
        path.poses = positions;
    
        pub.publish(path);
        return true;
    }
};

int main(int argc, char **argv){
    ros::init(argc, argv, "path_publisher");
    static_publisher sp;
    ros::spin();
}