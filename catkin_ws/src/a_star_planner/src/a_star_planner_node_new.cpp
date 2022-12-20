#include <iostream>
#include <cmath>
#include <cstdlib>
#include <vector>
#include <string>
#include <fstream>

// pcl
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
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Point.h>
#include "a_star_planner/AStar.hpp"

class AstarPlanner
{
    public:
        AstarPlanner(ros::NodeHandle& nh);        
        ~AstarPlanner();
        void CallbackGridMap(const nav_msgs::OccupancyGrid& msg);
        void CallbackObstacle(const sensor_msgs::PointCloud& msg);
        void CallbackOdom(const nav_msgs::Odometry& msg);
        void CallbackLookingAhead(const geometry_msgs::Point& msg);
        ros::NodeHandle nh_;
        
        ros::Subscriber subOccupancyGrid;
        ros::Subscriber subLookingAhead;
        ros::Subscriber subOdom;
        ros::Subscriber subObstacle;

        ros::Publisher pubAstarPath; 
        nav_msgs::Path m_LocalPath;
        geometry_msgs::PoseStamped m_GoalPose;
        bool bNewGoalPose;

        // For localization
        double target_x; //longitudinal
        double target_y; //lateral
        double start_x;
        double start_y;
        double origin_x;
        double origin_y;

        // For map 
        nav_msgs::OccupancyGrid map_sub;
        int grid_x_size;
        int grid_y_size;
        double resolution;
        
        // For transformation from local to global 
        Eigen::Matrix4f TF_matrix; 

};

AstarPlanner::AstarPlanner(ros::NodeHandle& nh) : nh_(nh), bNewGoalPose(false)
{
    subOccupancyGrid = nh_.subscribe("/repeat_map",1, &AstarPlanner::CallbackGridMap, this);
    subLookingAhead = nh_.subscribe("/looking_ahead_point", 1, &AstarPlanner::CallbackLookingAhead, this);
    subOdom = nh_.subscribe("/odom",1, &AstarPlanner::CallbackOdom, this);
    subObstacle = nh_.subscribe("/obstacle_pointcloud",1, &AstarPlanner::CallbackObstacle, this);
    pubAstarPath = nh_.advertise<nav_msgs::Path>("/Path_a_star", 1, true);
};

AstarPlanner::~AstarPlanner() 
{    
    ROS_INFO("AstarPlanner destructor.");
}


void AstarPlanner::CallbackGridMap(const nav_msgs::OccupancyGrid& msg)
{
    grid_x_size = msg.info.width;
    grid_y_size = msg.info.height;
    resolution  = msg.info.resolution;
    map_sub = msg;
}

void AstarPlanner::CallbackOdom(const nav_msgs::Odometry& msg)
{
    // get start point for path planning
    start_x = msg.pose.pose.position.x;
    start_y = msg.pose.pose.position.y;
    
    // Transfromation matrix 
     
    tf::Quaternion q;
    q.setX(msg.pose.pose.orientation.x);
    q.setY(msg.pose.pose.orientation.y);
    q.setZ(msg.pose.pose.orientation.z);
    q.setW(msg.pose.pose.orientation.w);
    tf::Matrix3x3 m(q);

    for(int i = 0; i <3; i++){
        tf::Vector3 row = m.getRow(i);
        for(int j=0; j<3; j++){
            TF_matrix.coeffRef(i,j) = row[j];
        }
    }

    for(int i=0; i<3; i++){TF_matrix.coeffRef(3, i) = 0;}
    TF_matrix.coeffRef(3,3) = 1;
    TF_matrix.coeffRef(0,3) = msg.pose.pose.orientation.x;
    TF_matrix.coeffRef(1,3) = msg.pose.pose.orientation.y;
    TF_matrix.coeffRef(2,3) = msg.pose.pose.orientation.z;
}

void AstarPlanner::CallbackLookingAhead(const geometry_msgs::Point& msg)
{
    target_x = msg.x;
    target_y = msg.y;
}

void AstarPlanner::CallbackObstacle(const sensor_msgs::PointCloud& msg)
{
    AStar::Generator generator;
    // Set 2d map size.
    generator.setWorldSize({grid_x_size, grid_y_size});
    // You can use a few heuristics : manhattan, euclidean or octagonal.
    generator.setHeuristic(AStar::Heuristic::euclidean);
    generator.setDiagonalMovement(true);

    int origin_grid_x = -origin_x / resolution + 1;
    int origin_grid_y = -origin_y / resolution + 1;
    int start_grid_x  = start_x / resolution + origin_grid_x;
    int start_grid_y  = start_y / resolution + origin_grid_y;
    int target_grid_x = (target_x) / resolution + origin_grid_x;
    int target_grid_y = (target_y) / resolution + origin_grid_y;
    
    // cut the goal pose in the grid map
    if(target_grid_y < 0)
        target_grid_y = 0;
    else if(target_grid_y > grid_y_size)
        target_grid_y = grid_y_size;    

    // add the collision 
    generator.clearCollisions();
    int buf_size = 1;
    // int check = 0;

    // // add map obstacle 
    // for (int grid_x = 0; grid_x < grid_x_size; grid_x++) {
    //     for (int grid_y = 0; grid_y < grid_y_size; grid_y++) {
    //         if (map_sub.data[grid_y * grid_x_size + grid_x] > 80) {
    //             generator.addCollision({grid_x, grid_y}, buf_size);
    //             check+=1;
    //             // pointTmp.x = grid_x* resolution +
    //             //                 resolution / 2 +
    //             //                 map_sub.info.origin.position.x;
    //             // pointTmp.y = grid_y* resolution +
    //             //                 resolution / 2 +
    //             //                 map_sub.info.origin.position.y;
    //         }
    //     }    
    // }
    // std::cout<< check << std::endl;

    
    // // Add laser obstacle 
    // for(auto point: msg.points)
    // {
    //     Eigen::Vector4f local_cor;
    //     local_cor[0] = point.x;
    //     local_cor[1] = point.y;
    //     local_cor[2] = 0;
    //     local_cor[3] = 1;
    //     Eigen::Vector4f global_cor;
    //     global_cor = TF_matrix*local_cor; 
    //     int grid_obstacle_x = global_cor[0]/resolution + origin_grid_x;
    //     int grid_obstacle_y = global_cor[1]/resolution + origin_grid_y;
    //     generator.addCollision({grid_obstacle_x, grid_obstacle_y}, buf_size);
    // }

    // //visualize the collision grid
    // sensor_msgs::PointCloud2 collisionCloudMsg;
    // pcl::toROSMsg(*cloud_in_ptr, collisionCloudMsg);
    // collisionCloudMsg.header.frame_id = "map";
    // collisionCloudMsg.header.stamp = ros::Time::now();
    // pubCollisionPoints.publish(collisionCloudMsg);

    // generate path 
    auto _path = generator.findPath({start_grid_x, start_grid_y}, {target_grid_x, target_grid_y});

    std::cout << "start : "<< start_grid_x << ", " << start_grid_y << std::endl;
    std::cout << "target : "<< target_grid_x << ", " << target_grid_y << std::endl;
    std::cout << "path size: " << _path.size() << std::endl;

    nav_msgs::Path AStartPathMsg;
    AStartPathMsg.header.stamp = ros::Time();
    AStartPathMsg.header.frame_id = "map";
    for(auto& coordinate : _path) {
        geometry_msgs::PoseStamped poseBuf;
        poseBuf.pose.position.x = (coordinate.x - origin_grid_x) * resolution; 
        poseBuf.pose.position.y = (coordinate.y - origin_grid_y) * resolution;
        AStartPathMsg.poses.push_back(poseBuf);
    }
    pubAstarPath.publish(AStartPathMsg);

}

int main(int argc, char* argv[])
{   
    ros::init(argc, argv, "a_star_planner");
    // for subscribe
    ros::NodeHandle nh;
    AstarPlanner planner(nh);

    ros::spin();
    return 0;

}
