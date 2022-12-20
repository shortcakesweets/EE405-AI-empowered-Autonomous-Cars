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

#include "a_star_planner/AStar.hpp"

class AstarPlanner
{
    public:
        AstarPlanner(ros::NodeHandle& nh);        
        ~AstarPlanner();
        void CallbackOccupancyGrid(const nav_msgs::OccupancyGrid& msg);
        void CallbackLocalPath(const nav_msgs::Path& msg);
        void CallbackGoalRviz(const geometry_msgs::PoseStamped& msg);

        ros::NodeHandle nh_;
        
    private:
        ros::Subscriber subOccupancyGrid;
        ros::Subscriber subLocalPath;
        ros::Subscriber subGoalPose;

        ros::Publisher pubAstarPath; 
        ros::Publisher pubCollisionPoints;

        nav_msgs::Path m_LocalPath;
        geometry_msgs::PoseStamped m_GoalPose;
        bool bNewGoalPose;
};

AstarPlanner::AstarPlanner(ros::NodeHandle& nh) : nh_(nh), bNewGoalPose(false)
{
    subOccupancyGrid = nh_.subscribe("/semantics/costmap_generator/occupancy_grid",1, &AstarPlanner::CallbackOccupancyGrid, this);
    subLocalPath = nh_.subscribe("/Path/LocalWaypoint/Fitting",1, &AstarPlanner::CallbackLocalPath, this);
    subGoalPose = nh_.subscribe("/move_base_simple/goal",1, &AstarPlanner::CallbackGoalRviz, this);

    pubAstarPath = nh_.advertise<nav_msgs::Path>("/Path/LocalWaypoint/a_star", 1, true);
    pubCollisionPoints = nh_.advertise<sensor_msgs::PointCloud2>("/PointCloud2/a_star_collision_points", 1, true);
};

AstarPlanner::~AstarPlanner() 
{    
    ROS_INFO("AstarPlanner destructor.");
}

void AstarPlanner::CallbackGoalRviz(const geometry_msgs::PoseStamped& msg)
{
    m_GoalPose = msg;
    bNewGoalPose = true;
}

void AstarPlanner::CallbackLocalPath(const nav_msgs::Path& msg)
{
    m_LocalPath = msg;

    int lookahead_idx = nh_.param("look_ahead_in_local_path", 5);
    if(msg.poses.size() > lookahead_idx && nh_.param("use_local_path_for_a_star", false))
    {
        m_GoalPose = m_LocalPath.poses.at(lookahead_idx);
        bNewGoalPose = true;
    }
}

void AstarPlanner::CallbackOccupancyGrid(const nav_msgs::OccupancyGrid& msg)
{
    if(!bNewGoalPose)
        return;

    double target_x = m_GoalPose.pose.position.x; //longitudinal
    double target_y = m_GoalPose.pose.position.y; //lateral

    int row, col, rotate_row, rotate_col;
    unsigned int grid_x_size = msg.info.width;
    unsigned int grid_y_size = msg.info.height;

    AStar::Generator generator;
    // Set 2d map size.
    generator.setWorldSize({grid_x_size, grid_y_size});
    // You can use a few heuristics : manhattan, euclidean or octagonal.
    generator.setHeuristic(AStar::Heuristic::euclidean);
    generator.setDiagonalMovement(false);

    generator.clearCollisions();

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in_ptr(new pcl::PointCloud<pcl::PointXYZI>);

    int origin_grid_x = -msg.info.origin.position.x / msg.info.resolution + 1;
    int origin_grid_y = -msg.info.origin.position.y / msg.info.resolution + 1;
    int target_grid_x = (target_x) / msg.info.resolution + origin_grid_x;
    int target_grid_y = (target_y) / msg.info.resolution + origin_grid_y;
    //cut the goal pose in the grid map
    if(target_grid_y < 0)
        target_grid_y = 0;
    else if(target_grid_y > grid_y_size)
        target_grid_y = grid_y_size;    

    //add the collision 
    int buf_size = nh_.param("grid_buf_size", 6);
    for (int grid_x = 0; grid_x < grid_x_size; grid_x++) {
        for (int grid_y = 0; grid_y < grid_y_size; grid_y++) {
            if (msg.data[grid_y * grid_x_size + grid_x] > 80) {
                generator.addCollision({grid_x, grid_y}, buf_size);

                pcl::PointXYZI pointTmp;
                pointTmp.x = grid_x* msg.info.resolution +
                                msg.info.resolution / 2 +
                                msg.info.origin.position.x;
                pointTmp.y = grid_y* msg.info.resolution+
                                msg.info.resolution / 2 +
                                msg.info.origin.position.y;

                cloud_in_ptr->points.push_back(pointTmp);
            }
        }    
    }
    //visualize the collision grid
    sensor_msgs::PointCloud2 collisionCloudMsg;
    pcl::toROSMsg(*cloud_in_ptr, collisionCloudMsg);
    collisionCloudMsg.header.frame_id = "base_link";
    collisionCloudMsg.header.stamp = ros::Time::now();
    pubCollisionPoints.publish(collisionCloudMsg);

    auto _path = generator.findPath({origin_grid_x, origin_grid_y}, {target_grid_x, target_grid_y});

    std::cout << "origin : "<< origin_grid_x << ", " << origin_grid_y << std::endl;
    std::cout << "target : "<< target_grid_x << ", " << target_grid_y << std::endl;
    std::cout << "path size: " << _path.size() << std::endl;

    nav_msgs::Path AStartPathMsg;
    AStartPathMsg.header.stamp = ros::Time();
    AStartPathMsg.header.frame_id = "base_link";
    for(auto& coordinate : _path) {
        geometry_msgs::PoseStamped poseBuf;
        poseBuf.pose.position.x = (coordinate.x - origin_grid_x) * msg.info.resolution; 
        poseBuf.pose.position.y = (coordinate.y - origin_grid_y) * msg.info.resolution;
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
