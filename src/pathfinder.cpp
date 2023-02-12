//General/ROS Includes
#include <ros/ros.h>
#include <ros/console.h>
#include <ros/callback_queue.h>
#include <vector>
#include <array>


#include <iostream>

#include <std_srvs/Empty.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include "pointcloud_process/occupancy_pos.h"

class pathfinder{
    private:
    ros::NodeHandle pf;
    ros::NodeHandle sv;

    ros::Subscriber sub;
    ros::Publisher pub;

    ros::ServiceServer server;
    ros::CallbackQueue grid_queue;
    
    geometry_msgs::PoseStamped goal;

    nav_msgs::OccupancyGridConstPtr occupancy_msg;
    
    public:
    pathfinder()
    {
        sub = pf.subscribe("/costmap_node/costmap/costmap", 1, &pathfinder::occupancy_callback, this);
        pub = pf.advertise<nav_msgs::Path>("/ozyegin/path", 1, true);
        server = sv.advertiseService("ozyegin/services/point_debug", &pathfinder::point_debug, this);
    }

    void occupancy_callback(const nav_msgs::OccupancyGridConstPtr& costmap_msg)
    {
        occupancy_msg = costmap_msg;
        return;
    }

    int get_index(std::array<int, 2> cell_indices)
    {
        return cell_indices[0] + cell_indices[1]*occupancy_msg->info.width; 
    }

    //Function implementations to turn map coordinates into cell indices.
    std::array<int, 2> pose_to_indices(geometry_msgs::PoseStamped input){
        std::array<int, 2> cell_indices = {0, 0};
        float local_x = input.pose.position.x - occupancy_msg->info.origin.position.x;
        float local_y = input.pose.position.y - occupancy_msg->info.origin.position.y;

        //Out-of-boundary checks
        if (!(0.0 < local_x < occupancy_msg->info.width*occupancy_msg->info.resolution && 0.0 < local_y < occupancy_msg->info.height*occupancy_msg->info.resolution)){
            ROS_INFO("pose_to_indices: sought point is out of occupancy grid bounds.");
        }

        //For loop to start from bottom right, and march cell by cell in respective x-y directions to pinpoint the cell coordinates of target point in local
        //coordinates.
        while ((1+cell_indices[1])*occupancy_msg->info.resolution < local_y){
            cell_indices[1] += 1;
        }
        while ((1+cell_indices[0])*occupancy_msg->info.resolution < local_x){
            cell_indices[0] += 1;
        }

        return cell_indices;
    }

    std::array<int, 2> pose_to_indices(float x, float y){
        std::array<int, 2> cell_indices = {0, 0};
        float local_x = x - occupancy_msg->info.origin.position.x;
        float local_y = y - occupancy_msg->info.origin.position.y;

        //Out-of-boundary checks
        if (!(0.0 < local_x < occupancy_msg->info.width*occupancy_msg->info.resolution && 0.0 < local_y < occupancy_msg->info.height*occupancy_msg->info.resolution)){
            ROS_INFO("pose_to_indices: sought point is out of occupancy grid bounds.");
        }

        //For loop to start from bottom right, and march cell by cell in respective x-y directions to pinpoint the cell coordinates of target point in local
        //coordinates.
        while ((1+cell_indices[1])*occupancy_msg->info.resolution < local_y){
            cell_indices[1] += 1;
        }
        while ((1+cell_indices[0])*occupancy_msg->info.resolution < local_x){
            cell_indices[0] += 1;
        }

        return cell_indices;
    }

    int point_cost(float x, float y)
    { 
        return occupancy_msg->data[get_index(pose_to_indices(x, y))];
    }

    bool point_debug(pointcloud_process::occupancy_posRequest& req, pointcloud_process::occupancy_posResponse& res)
    {
        ROS_INFO("Points received: point1(%f, %f), point2(%f, %f)", req.pose1_x, req.pose1_y, req.pose2_x, req.pose2_y);
        std::array<std::array<int, 2>, 2> request_points = {pose_to_indices(req.pose1_x, req.pose1_y), pose_to_indices(req.pose2_x, req.pose2_y)};
        std::vector<std::array<int, 2>> rasterized_cells = line_cells(request_points);
        for (int i = 0; i < rasterized_cells.size(); i++){
            if (occupancy_msg->data[get_index(rasterized_cells[i])] > 10){
                res.blocked = true;
                return true;
            }
        }
        res.blocked = false;
        return true;
    }


    double slope(std::array<int, 2> pose_1, std::array<int, 2> pose_2)
    {
        //Resolve infinite slope
        if (pose_1 == pose_2){
            return 0.0;
        }
        if (pose_1[0] == pose_2[0]){
            if (pose_1[1] > pose_2[1]){
                return -100.0;
            }
            else{
                return 100.0;
            }
        }

        //Slope calculation.    
        double delta_y = pose_2[1] - pose_1[1];
        double delta_x = pose_2[0] - pose_1[0];
        double slope = delta_y/delta_x;
        
        return slope;
    }

    //Returns a vector object that contains the coordinates of points between two lines.
    std::vector<std::array<int, 2>> line_cells(std::array<std::array<int, 2>, 2> indices)
    {
        
        std::array<std::array<int, 2>, 2> positions;

        if (indices[0][0] > indices[1][0]){
            positions[0][0] = indices[1][0];
            positions[0][1] = indices[1][1];
            positions[1][0] = indices[0][0];
            positions[1][1] = indices[0][1];
        }
        else{
            positions[0][0] = indices[0][0];
            positions[0][1] = indices[0][1];
            positions[1][0] = indices[1][0];
            positions[1][1] = indices[1][1];
        }
        std::array<int, 2> pen_pose = positions[0];        
        std::vector<std::array<int, 2>> drawn_cells;

        double true_slope =  slope(positions[0], positions[1]);

        drawn_cells.push_back((pen_pose));
        
        if (true_slope == 0.0){
            while (!(pen_pose == positions[1])){
                pen_pose[0] += 1;
                drawn_cells.push_back(pen_pose);
            }
        }

        else if (true_slope < 0.0){
            while (!(pen_pose == positions[1])){
                std::array<std::array<int, 2>, 2> points = {pen_pose, positions[1]};
                double traced_slope = slope(points[0], points[1]);
                if (traced_slope >= true_slope){
                    pen_pose[0] += 1;
                }
                else{
                    pen_pose[1] += -1;   
                }
                drawn_cells.push_back(pen_pose);
            }
        }

        else{
            while (!(pen_pose == positions[1])){
                std::array<std::array<int, 2>, 2> points = {pen_pose, positions[1]};
                double traced_slope = slope(points[0], points[1]);
                if (traced_slope > true_slope){
                    pen_pose[1] += 1;
                }
                else{
                    pen_pose[0] += 1;
                }
                drawn_cells.push_back(pen_pose);
            }
        }

        return drawn_cells;
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pathfinder");
    pathfinder toprak;
    ros::spin();
}