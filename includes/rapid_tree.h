#pragma once

#include <array>
#include <vector>
#include <memory>
#include <iterator>
#include <cmath>

#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include "occupancy_utility.h"

struct node{
    std::shared_ptr<node> parent = nullptr;
    std::array<int, 2> pose;
};

class RRT{
    nav_msgs::OccupancyGridConstPtr occupancy_msg;
    std::vector<std::shared_ptr<node>> node_list;
    float goal_radsqr;
    bool waypoints_set = false;

    int getClosestNodeIndex(std::array<int, 2> pose){
        
        //We will have to loop over every node in our tree except the goal, and keep in mind the index, because fuck memory management.
        float closestNodeDistance = distance_sqr(node_list[1]->pose, pose);
        int closestNodeIndex = 1;        

        for (int i = 1; i < node_list.size(); i++){

            //Move the inspected node into the stack frame.
            std::array<int, 2> candidateNodePose = node_list[i]->pose;
            float candidateNodeDistance = distance_sqr(candidateNodePose, pose);

            //If the indexed node is closer, change our opinion as to what nodes are closer.
            if (closestNodeDistance > candidateNodeDistance){
            
                closestNodeDistance = candidateNodeDistance;
                closestNodeIndex = i;
            
            }
        }

        return closestNodeIndex;
    }
    
    std::array<int, 2> calculated_position(std::array<int, 2> node_pose, std::array<int, 2> random_pose, float length){
        float delta_x = random_pose[0] - node_pose[0];
        float delta_y = random_pose[1] - node_pose[1];
        
        std::array<int, 2> resultPosition = {static_cast<int>(node_pose[0] + length*(delta_x/pow(pow(delta_x, 2) + pow(delta_y, 2), 0.5))), static_cast<int>(node_pose[1] + length*(delta_y/pow(pow(delta_x, 2) + pow(delta_y, 2), 0.5)))}; 
        
        if (0 > resultPosition[0]){
            resultPosition[0] = 0;
        }
        
        if (0 > resultPosition[1]){
            resultPosition[1] = 0;
        }

        if (resultPosition[0] > (occupancy_msg->info.width - 1)){
            resultPosition[0] = occupancy_msg->info.width - 1;
        }

        if (resultPosition[1] > (occupancy_msg->info.height - 1)){
            resultPosition[1] = occupancy_msg->info.height - 1;
        }

        return resultPosition;
    }

    public:
    RRT(){
        node_list.reserve(1024);
        occupancy_msg = nav_msgs::OccupancyGridConstPtr(new nav_msgs::OccupancyGrid);
    }
    void set_occupancy(const nav_msgs::OccupancyGridConstPtr occupancy_input){
        occupancy_msg = occupancy_input;
    }
    void set_goal_radius(float radi){
        goal_radsqr = pow(radi, 2);
    }
    void set_waypoints(float x_goal, float y_goal){

        if (waypoints_set){
            return;
        }

        std::array<int, 2> goal_pose = pose_to_indices(x_goal, y_goal, occupancy_msg);

        node_list.emplace_back(std::make_shared<node>());
        node_list.emplace_back(std::make_shared<node>());

        int rootX = occupancy_msg->info.width/2;
        int rootY = occupancy_msg->info.height/2;

        node_list[0]->pose = goal_pose;
        node_list[1]->pose = {rootX, rootY};

        waypoints_set = true;
    }
    bool generate_node(float length){
        while (true){

            std::array<int, 2> generated_pose = random_indices(occupancy_msg);
            std::array<int, 2> node_position;
            int closestNodeIndex = getClosestNodeIndex(generated_pose);

            node_position = calculated_position(node_list[closestNodeIndex]->pose, generated_pose, length);

            std::cout << node_position[0] << ", " << node_position[1] << std::endl;

            if (check_collision(node_list[closestNodeIndex]->pose, node_position, occupancy_msg, 50.0)){
                continue;
            }

            node_list.emplace_back(std::make_shared<node>());   
            node_list[node_list.size() - 1]->parent = node_list[closestNodeIndex];
            node_list[node_list.size() - 1]->pose = node_position;
            
            if ((distance_sqr(node_position, node_list[0]->pose) < goal_radsqr) && !(check_collision(node_position, node_list[0]->pose, occupancy_msg, 50))){
                node_list[0]->parent = node_list[node_list.size() - 1];
                return true;
            }

            return false;
        }
    }

    nav_msgs::Path path_to_goal(){
        std::vector<std::array<int, 2>> pathWaypoints;

        while (true){
            if (generate_node(30.0)){ 
                std::shared_ptr<node> current_node = node_list[0];
                while (current_node != node_list[1]){
                    pathWaypoints.insert(pathWaypoints.begin(), current_node->pose);
                    current_node = current_node->parent;
                }
                break;
            }
        }

        std::vector<geometry_msgs::PoseStamped> pathData;
        for (int i = 0; i < pathWaypoints.size(); i++){
            geometry_msgs::PoseStamped pathWaypoint = indices_to_pose(pathWaypoints[i], occupancy_msg);
            pathData.push_back(pathWaypoint);
        }

        nav_msgs::Path path;
        path.header.frame_id = "map";
        path.poses = pathData;
        return path;
    }
};