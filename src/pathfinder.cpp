//General/ROS Includes
#include <ros/ros.h>
#include <ros/callback_queue.h>

#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

#include "pointcloud_process/occupancy_pos.h"
#include "../includes/occupancy_utility.h"
#include "../includes/rapid_tree.h"

class pathfinder{
    private:
    ros::NodeHandle pf;
    ros::NodeHandle sv;

    ros::Subscriber sub;
    ros::Publisher pub;

    ros::ServiceServer server;
    ros::CallbackQueue grid_queue;

    RRT world_tree;

    nav_msgs::OccupancyGridConstPtr occupancy_msg;
    
    public:
    pathfinder()
    {
        sub = pf.subscribe("/costmap_node/costmap/costmap", 1, &pathfinder::occupancy_callback, this);
        pub = pf.advertise<nav_msgs::Path>("/ozyegin/path", 1, true);
        server = sv.advertiseService("ozyegin/services/pathfind", &pathfinder::pathfind, this);
        world_tree.set_goal_radi_sqr(625.0);
    }

    void occupancy_callback(const nav_msgs::OccupancyGridConstPtr& costmap_msg)
    {
        occupancy_msg = costmap_msg;
        world_tree.set_occupancy(occupancy_msg);
        return;
    }

    bool pathfind(pointcloud_process::occupancy_posRequest& req, pointcloud_process::occupancy_posResponse& res)
    {
        std::array<int, 2> pose_start = pose_to_indices(req.pose1_x, req.pose1_y, occupancy_msg);
        std::array<int, 2> pose_end = pose_to_indices(req.pose2_x, req.pose2_y, occupancy_msg);
        world_tree.set_waypoints(pose_start, pose_end);
        nav_msgs::Path path_to_goal = world_tree.path_to_goal();
        pub.publish(path_to_goal);
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