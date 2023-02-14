#pragma once

#include <array>
#include <vector>
#include <cmath>

#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include "occupancy_utility.h"

class node{
    private:
    node* parent = nullptr;
    std::vector<node*> children;
    std::array<int, 2> pose;

    
    void add_child(node* child_pointer){
        children.push_back(child_pointer);
    }

    public:
    void set_pose(std::array<int, 2> position){
        pose = position;
    }

    void set_parent(node* parent_pointer){
        parent = parent_pointer;
        parent_pointer->add_child(this);
    }

    std::array<int, 2> get_pose(){
        return pose;
    }

    node* get_parent(){
        return parent;
    }

    std::vector<node*> get_children(){
        return children;
    }
};

class RRT{
    private:
    node pose_start;
    node pose_end;
    std::vector<node> node_list = {pose_start, pose_end};
    float goal_radius_sqr;
    float exploration_bias;
    nav_msgs::OccupancyGridConstPtr occupancy_grid;
    
    int get_closest_index(std::array<int, 2> pose)
    {
        float least_distance_sqr = distance_sqr(pose, pose_start.get_pose());
        int index;

        for (int i = 0; i < node_list.size(); i++)
        {
            float distance = distance_sqr(pose, node_list[i].get_pose());
            if (least_distance_sqr > distance)
            {    
                least_distance_sqr = distance;
                index = i;
            }
        }

        return index;
    }

    bool check_intersection(std::array<int, 2> pose1, std::array<int, 2> pose2)
    {
        std::vector<std::array<int, 2>> drawn_cells = line_cells(pose1, pose2);
        for (int i = 0; i < drawn_cells.size(); i++){
            if (10 < occupancy_grid->data[get_index(drawn_cells[i], occupancy_grid)]){
                return true;
            }
        }
        return false;
    }

    bool generate_node()
    {
        while (true)
        {
            srand(time(0));
            int random_x = rand() % occupancy_grid->info.height;
            int random_y = rand() % occupancy_grid->info.width;
            std::array<int, 2> random_pose = {random_x, random_y};
            std::array<int, 2> candidate_pose;
            int closest_index = get_closest_index(random_pose);

            int delta_x = random_x - node_list[closest_index].get_pose()[0];
            int delta_y = random_y - node_list[closest_index].get_pose()[1];

            candidate_pose[0] = node_list[closest_index].get_pose()[0] + 20*floor((delta_x)/pow(pow(delta_x, 2) + pow(delta_y, 2), 0.5));
            candidate_pose[1] = node_list[closest_index].get_pose()[1] + 20*floor((delta_y)/pow(pow(delta_x, 2) + pow(delta_y, 2), 0.5));

            if (check_intersection(node_list[closest_index].get_pose(), candidate_pose))
            {
                continue;
            }

            node candidate;
            candidate.set_pose(candidate_pose);
            candidate.set_parent(&node_list[closest_index]);

            node_list.push_back(candidate);

            if (distance_sqr(pose_end.get_pose(), candidate.get_pose()) < goal_radius_sqr && !(check_intersection(pose_end.get_pose(), candidate.get_pose())))
            {
                pose_end.set_parent(&node_list.back());
                return true;
            }

            return false;
        }
    }

    public:
    void set_goal_radi_sqr(float radi_sqr){
        goal_radius_sqr = radi_sqr;
    }

    void set_waypoints(std::array<int, 2> initial, std::array<int, 2> final)
    {
        pose_start.set_pose(initial);
        node_list.push_back(pose_start);
        pose_end.set_pose(final);
    }

    void set_occupancy(nav_msgs::OccupancyGridConstPtr occupancy_msg)
    {
        occupancy_grid = occupancy_msg;
    }

    nav_msgs::Path path_to_goal(){
        nav_msgs::Path pathway;
        while(true){
            if(generate_node()){
                break;
            }
        }

        std::vector<geometry_msgs::PoseStamped> poses;
        node* current_node = &pose_end;
        while (current_node != &pose_start){
            geometry_msgs::PoseStamped path_node_position;
            path_node_position.pose.position.x = current_node->get_pose()[0];
            path_node_position.pose.position.y = current_node->get_pose()[1];
            path_node_position.header.frame_id = "map";
            poses.insert(poses.begin(), path_node_position);
        }

        pathway.poses = poses;
        return pathway;
    }

};