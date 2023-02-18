#pragma once

#include <array>
#include <vector>
#include <geometry_msgs/PoseStamped.h>
#include <random>
#include <ros/ros.h>
#include <ros/console.h>
#include <nav_msgs/OccupancyGrid.h>

std::vector<std::array<int, 2>> line_cells(std::array<std::array<int, 2>, 2> indices) {
    std::vector<std::array<int, 2>> cells;

    int dx = indices[1][0] - indices[0][0];
    int dy = indices[1][1] - indices[0][1];

    int steps = std::max(abs(dx), abs(dy));

    float x_step = static_cast<float>(dx) / steps;
    float y_step = static_cast<float>(dy) / steps;

    float x = indices[0][0];
    float y = indices[0][1];

    for (int i = 0; i < steps + 1; i++) {
        cells.push_back({ static_cast<int>(round(x)), static_cast<int>(round(y)) });
        x += x_step;
        y += y_step;
    }

    return cells;   
}

std::vector<std::array<int, 2>> line_cells(std::array<int, 2> start, std::array<int, 2> end) {
    std::vector<std::array<int, 2>> cells;

    int dx = end[0] - start[0];
    int dy = end[1] - start[1];

    int steps = std::max(abs(dx), abs(dy));

    float x_step = static_cast<float>(dx) / steps;
    float y_step = static_cast<float>(dy) / steps;

    float x = start[0];
    float y = start[1];

    for (int i = 0; i < steps + 1; i++) {
        cells.push_back({ static_cast<int>(round(x)), static_cast<int>(round(y)) });
        x += x_step;
        y += y_step;
    }

    return cells;   
}

int get_index(std::array<int, 2> cell_indices, const nav_msgs::OccupancyGridConstPtr occupancy_msg)
{
    return cell_indices[0] + cell_indices[1]*occupancy_msg->info.width; 
}

std::array<int, 2> pose_to_indices(const geometry_msgs::PoseStamped input, nav_msgs::OccupancyGridConstPtr occupancy_msg){
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

std::array<int, 2> pose_to_indices(float x, float y, const nav_msgs::OccupancyGridConstPtr occupancy_msg){
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

geometry_msgs::PoseStamped indices_to_pose(std::array<int, 2> indices, const nav_msgs::OccupancyGridConstPtr occupancy_msg){
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.pose.position.x = (0.5 + indices[0])*occupancy_msg->info.resolution + occupancy_msg->info.origin.position.x;
    pose.pose.position.y = (0.5 + indices[1])*occupancy_msg->info.resolution + occupancy_msg->info.origin.position.y;
    pose.pose.position.z = 0.0;
    pose.pose.orientation.w = 1.0;
    return pose; 
}

int point_cost(float x, float y, const nav_msgs::OccupancyGridConstPtr occupancy_msg)
{ 
    return occupancy_msg->data[get_index(pose_to_indices(x, y, occupancy_msg), occupancy_msg)];
}

int point_cost(std::array<int, 2> position, const nav_msgs::OccupancyGridConstPtr occupancy_msg)
{ 
    return occupancy_msg->data[get_index(position, occupancy_msg)];
}

float distance_sqr(std::array<int, 2> pose1, std::array<int, 2> pose2){
    return pow(pose1[0] - pose2[0], 2) + pow(pose1[1] - pose2[1], 2);
}

std::array<int, 2> random_indices(const nav_msgs::OccupancyGridConstPtr occupancy_msg){
    std::random_device rd;
    std::mt19937 mt(rd());
    std::uniform_int_distribution<int> distx(0, occupancy_msg->info.width - 1);
    std::uniform_int_distribution<int> disty(0, occupancy_msg->info.height - 1);
    int random_x = distx(mt);
    int random_y = disty(mt);
    std::array<int, 2> indices = {random_x, random_y};
    return indices;
}

bool check_collision(std::array<int, 2> pose1, std::array<int, 2> pose2, const nav_msgs::OccupancyGridConstPtr occupancy_msg, int threshold){
    std::vector<std::array<int, 2>> line_drawn = line_cells(pose1, pose2);
    for (int i = 0; i < line_drawn.size(); i++){
        if (occupancy_msg->data[get_index(line_drawn[i], occupancy_msg)] > threshold){
            return true;
        }
    }
    return false;
}