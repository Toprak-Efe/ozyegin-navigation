//General/ROS Includes
#include <ros/ros.h>
#include <ros/console.h>
#include <ros/callback_queue.h>

#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_srvs/SetBool.h>
#include "pointcloud_process/occupancy_pos.h"
#include "../includes/rapid_tree.h"
#include "../includes/occupancy_utility.h"

class pathfinder{
    private:
    ros::NodeHandle pf;
    ros::NodeHandle sv1;
    ros::NodeHandle sv2;

    ros::Subscriber sub;
    ros::Publisher pub;

    ros::ServiceServer server1;
    ros::ServiceServer server2;
    nav_msgs::OccupancyGridConstPtr occupancy_store;
    nav_msgs::Path pathStore;
    
    public:
    pathfinder()
    {
        sub = pf.subscribe("/costmap_node/costmap/costmap",1, &pathfinder::occupancy_callback, this);
        pub = pf.advertise<nav_msgs::Path>("/ozyegin/path", 1, false);

        server1 = sv1.advertiseService("ozyegin/services/pathfind", &pathfinder::pathfind, this);
        server2 = sv2.advertiseService("ozyegin/services/check_path", &pathfinder::check_path, this);
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
        pathStore = pathToGoal;
        res.published = true;
        
        return true;
    }

    bool check_path(std_srvs::SetBoolRequest& req, std_srvs::SetBoolResponse& res){
        ROS_INFO("Checking path.");
        for (int i = 0; i < pathStore.poses.size() - 1; i++){
            std::array<int, 2> pose1 = pose_to_indices(pathStore.poses[i], occupancy_store);
            std::array<int, 2> pose2 = pose_to_indices(pathStore.poses[i+1], occupancy_store);
            ROS_INFO("Checking path waypoint.");
            if (check_collision(pose1, pose2, occupancy_store, 50)){
                ROS_INFO("pathfinder: Collision detected.");
                res.success = 1;
                return true;
            }
        }
        res.success = 0;
        return true;
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pathfinder");
    pathfinder toprak;
    ros::spin();
}