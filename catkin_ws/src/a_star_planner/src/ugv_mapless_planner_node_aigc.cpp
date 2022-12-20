#include <iostream>
#include <cmath>
#include <cstdlib>
#include <vector>
#include <string>
#include <fstream>

// pcl
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "tf/transform_datatypes.h"
#include <tf2_msgs/TFMessage.h>
#include <tf/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
// ros
#include "ros/ros.h"
#include <nav_msgs/GetMap.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt8MultiArray.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/OccupancyGrid.h>

#include "AStar.hpp"

#define GRID_HEIGHT 41
#define GRID_WIDTH 41
#define GRID_RES 0.15
#define NUM_GRID 41*41
#define safety_row_cell 0
#define safety_col_cell 0

using namespace std;

AStar::Generator generator;
nav_msgs::Path path;
nav_msgs::Path path_origin;
std::vector<uint> Cos;
int target_idx = 0;
int grid_row;
int grid_col;
bool is_visual_servoing;
double path_row_x, path_col_y;
double odom_x, odom_y;
double roll, pitch, yaw;
double world_x, world_y;
bool pub = true;
bool map_update_flag = true;
int origin_row, origin_col, rotate_row, rotate_col;
bool path_pub_flag= true;
void odom_callback(const nav_msgs::Odometry& msg){

      tf::Quaternion quat_rtk;
      tf::quaternionMsgToTF(msg.pose.pose.orientation,quat_rtk);
      tf::Matrix3x3(quat_rtk).getRPY(roll,pitch,yaw);
      odom_x = msg.pose.pose.position.x;
      odom_y = msg.pose.pose.position.y;
}



void driverRegionCallback(const nav_msgs::OccupancyGrid& msg)
{
        if (map_update_flag==true){

            generator.clearCollisions();
            Cos.clear();
            // Set default occupied grid
            // for(int i =1; i<=GRID_HEIGHT; i++)
            // {
            //     for(int j =1; j<=GRID_WIDTH; j++)
            //     {
            //         // if(j <= (GRID_HEIGHT-i) || j >= (GRID_HEIGHT+i))
            //         // {
            //         //     generator.addCollision({i, j}); // default region
            //         // }
            //         // else
            //         // {
            //         //     if( msg->data[(i-1)*(i-1)+(j-i+1)] == false )
            //         //     {
            //         //         generator.addCollision({i, j});
            //         //     }

            //         // }
            //     }
            // }
            for(int i =0 ; i < msg.data.size(); i++){
                Cos.push_back(msg.data[i]);
                if(msg.data[i] == 0){
                    origin_row = int(i/GRID_WIDTH);
                    origin_col = i%GRID_WIDTH;
                    rotate_row = origin_col;
                    rotate_col = GRID_HEIGHT - origin_row - 1;
                    if (rotate_row != int(GRID_HEIGHT/2)){  
                        generator.addCollision({rotate_row, rotate_col});
                    }
                    if (rotate_col != int(GRID_WIDTH/2)){  
                        generator.addCollision({rotate_row, rotate_col});
                    }
                //     // std::cout<<msg->data.size()<<std::endl;
                }
            }
            map_update_flag = false;
            // grid_m = msg;
        }

}

void targetIdxCallback(const std_msgs::Int16::ConstPtr& msg)
{
    if (map_update_flag == false){
    path_pub_flag = true;    
    ros::NodeHandle nh;
    ros::Publisher pub_path = nh.advertise<nav_msgs::Path>("/path_astar",1);
    ros::Publisher pub_path_origin = nh.advertise<nav_msgs::Path>("/path_astar_origin",1);
    ros::Publisher pub_goal_marker = nh.advertise<visualization_msgs::Marker>("/Goal_marker",1);
    ros::Publisher pub_wpt = nh.advertise<std_msgs::Float32MultiArray>("/scout/astar_path_info",1);
    // is_visual_servoing = true;
     visualization_msgs::Marker points;
     geometry_msgs::Point p;
     std_msgs::Float32MultiArray tar_p;
        target_idx = msg->data;
        // std::cout<<"target_idx"<<target_idx<<std::endl;
        // std::cout<<"GRID_WIDTH"<<GRID_WIDTH<<std::endl;
        grid_row = int((target_idx)/GRID_WIDTH);
        // std::cout<<"grid_row"<<grid_row<<std::endl;
        grid_col = (target_idx)-(int((target_idx)/GRID_WIDTH))*GRID_WIDTH;
        // std::cout<<"grid_col"<<grid_col<<std::endl;
        generator.removeCollision({int(GRID_HEIGHT/2),int(GRID_WIDTH/2)});
        for (int r_idx= -safety_row_cell; r_idx<=safety_row_cell; r_idx++){
            for (int r_idx_= -safety_col_cell; r_idx_<=safety_col_cell; r_idx_++){
                generator.removeCollision({grid_row+r_idx, grid_col+r_idx_});
            }
        }
        generator.removeCollision({grid_row, grid_col});
        // std::cout<<"target_idx"<<target_idx<<std::endl;
        // std::cout<<"grid_row"<<grid_row<<std::endl;
        // std::cout<<"grid_col"<<grid_col<<std::endl;
        auto _path = generator.findPath({int(GRID_HEIGHT/2),int(GRID_WIDTH/2)}, {grid_row,grid_col}, Cos);

        path.poses.clear();
        path.header.stamp = ros::Time();
        path.header.frame_id = "odom";
        path_origin.poses.clear();
        path_origin.header.stamp = ros::Time();
        path_origin.header.frame_id = "odom";
        for(auto& coordinate : _path) {
            // std::cout << "start"<< "\n";
            geometry_msgs::PoseStamped pose;
            geometry_msgs::PoseStamped pose_origin;
            pose.header.stamp = ros::Time();
            pose.header.frame_id = "odom";
            pose_origin.header.stamp = ros::Time();
            pose_origin.header.frame_id = "odom";
            path_row_x = (coordinate.x - int(GRID_HEIGHT/2)) * GRID_RES;
            path_col_y = -(coordinate.y - int(GRID_WIDTH/2)) * GRID_RES;
            world_x = path_row_x*cos(yaw) - path_col_y*sin(yaw);
            world_y = path_row_x*sin(yaw) + path_col_y*cos(yaw);
            pose.pose.position.x = odom_x + world_x;
            pose.pose.position.y = odom_y + world_y;
            pose.pose.position.z = 0;
            pose_origin.pose.position.x = path_row_x;
            pose_origin.pose.position.y = path_col_y;
            pose_origin.pose.position.z = 0;
            path.poses.push_back(pose);
            path_origin.poses.push_back(pose_origin);
        }
        pub_path.publish(path);
        pub_path_origin.publish(path_origin);
        if (path.poses.size()<=3){
            points.header.frame_id = "odom";
            points.header.stamp= ros::Time::now();
            points.ns="Goal_Pt";
            points.id = 0;
            points.type = visualization_msgs::Marker::POINTS;
            points.scale.x = 0.2;
            points.scale.y = 0.2;
            points.color.r = 1.0f;
            points.color.a = 1.0;

            geometry_msgs::Point p;
            std::vector<double> vec1;
            vec1.clear();
            p.x = path.poses[0].pose.position.x;
            p.y = path.poses[0].pose.position.y;
            vec1 = {p.x, p.y, 0, 0, 0};
            points.points.push_back(p);
            //points.pose.push_back(p);
            pub_goal_marker.publish(points);
            tar_p.data.clear();
            tar_p.data.insert(tar_p.data.end(), vec1.begin(), vec1.end());
            pub_wpt.publish(tar_p);
        }
        else{
            points.header.frame_id = "odom";
            points.header.stamp= ros::Time::now();
            points.ns="Goal_Pt";
            points.id = 0;
            points.type = visualization_msgs::Marker::POINTS;
            points.scale.x = 0.2;
            points.scale.y = 0.2;
            points.color.r = 1.0f;
            points.color.a = 1.0;

            geometry_msgs::Point p;
            std::vector<double> vec1;
            vec1.clear();
            p.x = path.poses[path.poses.size()-3].pose.position.x;
            p.y = path.poses[path.poses.size()-3].pose.position.y;
            vec1 = {p.x, p.y, 0, 0, 0};
            points.points.push_back(p);
            //points.pose.push_back(p);
            pub_goal_marker.publish(points);
            tar_p.data.clear();
            tar_p.data.insert(tar_p.data.end(), vec1.begin(), vec1.end());
            pub_wpt.publish(tar_p);
        }
        map_update_flag = true;
    }



    // // direct to target
    // generator.clearCollisions();
    // auto _path_direct = generator.findPath({0,GRID_WIDTH/2}, {grid_row,grid_col});

    // if(_path.size() != _path_direct.size())
    // {
    //     cout << "visual servoing unavailable" << endl;
    //     is_visual_servoing = false;
    // }
    // else
    // {
    //     for(int i =0; i<_path_direct.size(); i++)
    //     {
    //         if(_path[i].x != _path_direct[i].x || _path[i].y != _path_direct[i].y) 
    //         {
    //             cout << "visual servoing unavailable" << endl;
    //             is_visual_servoing = false;
    //             break;
    //         }
    //     }
    // }
}

int main(int argc, char* argv[])
{   
    // Set 2d map size.
    generator.setWorldSize({GRID_HEIGHT,GRID_WIDTH});
    // You can use a few heuristics : manhattan, euclidean or octagonal.
    generator.setHeuristic(AStar::Heuristic::euclidean);
    generator.setDiagonalMovement(true);    

    ros::init(argc, argv, "ugv_mapless_planner_node");

    ros::Time::init();

    // for subscribe
    ros::NodeHandle nh;
    ros::Subscriber sub_grid = nh.subscribe("/distance_map",1,driverRegionCallback);
    ros::Subscriber sub_target_idx = nh.subscribe("/target_index",1,targetIdxCallback);
    ros::Subscriber odometry = nh.subscribe("/Odometry/final",1,odom_callback);

    // for publish
    ros::NodeHandle nh_pub;
    ros::Publisher pub_path = nh.advertise<nav_msgs::Path>("/path_astar",1);
    ros::Publisher pub_path_origin = nh.advertise<nav_msgs::Path>("/path_astar_origin",1);
    ros::Publisher pub_vs_mode = nh.advertise<std_msgs::Bool>("/is_vs_mode",1);
    ros::Publisher pub_goal_marker = nh.advertise<visualization_msgs::Marker>("/Goal_marker",1);
    ros::Publisher pub_wpt = nh.advertise<std_msgs::Float32MultiArray>("/scout/astar_path_info",1);

    ros::spin();
    return 0;

}
