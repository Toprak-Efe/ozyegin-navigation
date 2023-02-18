//General/ROS Includes
#include <ros/ros.h>
#include <ros/console.h>
#include <ros/callback_queue.h>

#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

#include "pointcloud_process/occupancy_pos.h"
#include "../includes/rapid_tree.h"

class pathfinder{
    private:
    ros::NodeHandle pf;
    ros::NodeHandle sv;

    ros::Subscriber sub;
    ros::Publisher pub;

    ros::ServiceServer server;
    nav_msgs::OccupancyGridConstPtr occupancy_store;
    
    public:
    pathfinder()
    {
        sub = pf.subscribe("/costmap_node/costmap/costmap",1, &pathfinder::occupancy_callback, this);
        pub = pf.advertise<nav_msgs::Path>("/ozyegin/path", 1, false);
        server = sv.advertiseService("ozyegin/services/pathfind", &pathfinder::pathfind, this);
        ROS_INFO("pathfinder: object initialized.");
    }

    void occupancy_callback(const nav_msgs::OccupancyGridConstPtr& costmap_msg)
    {   
        std::cout << "pathfinder: occupancy detected." << costmap_msg->info.resolution << std::endl;
        occupancy_store = costmap_msg;
        std::cout << "pathfinder: occupancy retrieved." << std::endl;
    }

    bool pathfind(pointcloud_process::occupancy_posRequest& req, pointcloud_process::occupancy_posResponse& res)
    {
        ROS_INFO("pathfinder: service initializing.");
        RRT world_tree;
        world_tree.set_occupancy(occupancy_store);
        world_tree.set_waypoints(req.pose_x, req.pose_y);
        world_tree.set_goal_radius(25.0);
        
        nav_msgs::Path pathToGoal = world_tree.path_to_goal();
        ROS_INFO("pathfinder: path generated.");

        pub.publish(pathToGoal);
        res.published = true;
        
        return true;
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pathfinder");
    pathfinder toprak;
    ros::spin();
}