//General/ROS Includes
#include <ros/ros.h>
#include <ros/console.h>
#include <ros/callback_queue.h>

#include <iostream>

#include <std_srvs/Empty.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

class pathfinder{
    private:
    ros::NodeHandle pf;
    ros::NodeHandle sv;

    ros::Subscriber sub;
    ros::Publisher pub;

    ros::ServiceServer server;
    ros::CallbackQueue grid_queue;
    
    geometry_msgs::PoseStamped goal;
    
    public:
    pathfinder(){
        sub = pf.subscribe("/costmap_node/costmap/costmap", 1, &occupancy_callback, this);
        pub = pf.advertise<nav_msgs::Path>("/ozyegin/path", 1, true);
    }

    void occupancy_callback(const nav_msgs::OccupancyGridConstPtr& costmap_msg){
        return;
    }

    int point_to_cell(float pose_x, float pose_y, const nav_msgs::OccupancyGridConstPtr costmap_msg){
        //The distance between two adjacent cells.
        float resolution = costmap_msg->info.resolution;

        float height = costmap_msg->info.height*resolution;
        float width = costmap_msg->info.width*resolution;

        double costmap_x =  costmap_msg->info.origin.position.x;
        double costmap_y = costmap_msg->info.origin.position.y;

        float local_x = pose_x - costmap_x;
        float local_y = pose_y - costmap_y;

        //Out-of-boundary checks
        if (!(-costmap_x < local_x < costmap_x && -costmap_y < local_y < costmap_y)){
            ROS_INFO("Point out of bounds.");
            return 0;
        }

        int cell_indices[2] = {0, 0};

        //For loop to march from top-left to the bottom right corner of the cell our point is confided in.
        //Top-left-most coordinates: (-width/2, height/2)
        //Marching step will be resolution. First march downwards until you are superceded by local_y,
        //Take the number of steps, remove 1, that
        while ((1 + cell_indices[1])*resolution > local_y){
            cell_indices[1] += 1;
        }
        while ((1 + cell_indices[0])*resolution > local_x){
            cell_indices[0] += 1;
        }

        int index = cell_indices[0] + cell_indices[1]*pose_y;

        return index;
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pathfinder");
    pathfinder toprak;
    ros::spin();
}