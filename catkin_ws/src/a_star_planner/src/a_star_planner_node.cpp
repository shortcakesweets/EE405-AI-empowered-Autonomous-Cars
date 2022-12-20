#include <iostream>
#include <cmath>
#include <cstdlib>
#include <vector>
#include <string>

// pcl
#include <pcl/io/pcd_io.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <pcl/filters/approximate_voxel_grid.h>

// ros
#include "ros/ros.h"
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/PointCloud.h>
#include "a_star_planner/AStar.hpp"

pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in_ptr(new pcl::PointCloud<pcl::PointXYZI>); 
pcl::ApproximateVoxelGrid<pcl::PointXYZI> avg_filter;

class AstarPlanner
{
    public:
        AstarPlanner(ros::NodeHandle& nh);        
        ~AstarPlanner();
        void CallbackOccupancyGrid(const nav_msgs::OccupancyGrid& msg);
        void CallbackGoalRviz(const geometry_msgs::PoseStamped& msg);
        void CallbackLookAhead(const geometry_msgs::Point& msg);
        void CallbackOdom(const nav_msgs::Odometry& msg);
        void CallbackObstacle(const sensor_msgs::PointCloud& msg);

        ros::NodeHandle nh_;
        
        ros::Subscriber subOccupancyGrid;
        ros::Subscriber subLocalPath;
        ros::Subscriber subGoalPose;
        ros::Subscriber subLookAhead;
        ros::Subscriber subOdom;
        ros::Subscriber subObstacle;

        ros::Publisher pubAstarPath; 
        ros::Publisher pubCollisionPoints;

        nav_msgs::Path m_LocalPath;
        geometry_msgs::PoseStamped m_GoalPose;
        bool bNewGoalPose;
        bool HaveObstacle;
        
        // For target point
        double target_x = 0;
        double target_y = 0; 

        // For start point
        double start_x = 0;
        double start_y = 0;

        // For transformation from local to global 
        Eigen::Matrix4f TF_matrix; 
        tf::TransformListener listener;
        tf::StampedTransform transform;


        // For map
        double origin_x;
        double origin_y;
        unsigned int grid_x_size;
        unsigned int grid_y_size;
        double resolution;      
        nav_msgs::OccupancyGrid map_sub;  
        
        // For obstacle
        sensor_msgs::PointCloud obstacle;
        
     

};

AstarPlanner::AstarPlanner(ros::NodeHandle& nh) : nh_(nh), bNewGoalPose(false)
{
    subOccupancyGrid = nh_.subscribe("/road",1, &AstarPlanner::CallbackOccupancyGrid, this);
    subLookAhead = nh_.subscribe("/looking_ahead_point",1, &AstarPlanner::CallbackLookAhead, this);
    subGoalPose = nh_.subscribe("/move_base_simple/goal",1, &AstarPlanner::CallbackGoalRviz, this);
    subOdom = nh_.subscribe("/odom",1, &AstarPlanner::CallbackOdom, this);
    subObstacle = nh_.subscribe("/obstacle_pointcloud",1, &AstarPlanner::CallbackObstacle, this);

    pubAstarPath = nh_.advertise<nav_msgs::Path>("/A_star_path", 1, true);
    // pubCollisionPoints = nh_.advertise<sensor_msgs::PointCloud2>("/PointCloud2/a_star_collision_points", 1, true);
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

void AstarPlanner::CallbackObstacle(const sensor_msgs::PointCloud& msg)
{
    obstacle = msg;
    if (obstacle.points.size()>0){HaveObstacle = true;}
    else {HaveObstacle = false;}
}

void AstarPlanner::CallbackOdom(const nav_msgs::Odometry& msg)
{
    
    // Transfromation 
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
    TF_matrix.coeffRef(0,3) = msg.pose.pose.position.x;
    TF_matrix.coeffRef(1,3) = msg.pose.pose.position.y;
    TF_matrix.coeffRef(2,3) = 0;
    TF_matrix.coeffRef(3,3) = 1;

    std::cout<<"ok odom"<<std::endl;
}


void AstarPlanner::CallbackLookAhead(const geometry_msgs::Point& msg)
{
    // get start point for path planning
    start_x = TF_matrix.coeffRef(0,3);
    start_y = TF_matrix.coeffRef(1,3);
    
    target_x = msg.x;
    target_y = msg.y;
    std::cout << "size pcl " << obstacle.points.size() << std::endl; 
    if (!HaveObstacle)
        return;

    int origin_grid_x = -origin_x / resolution + 1;
    int origin_grid_y = -origin_y / resolution + 1;
    int start_grid_x  = start_x / resolution + origin_grid_x;
    int start_grid_y  = start_y / resolution + origin_grid_y;
    int target_grid_x = (target_x) / resolution + origin_grid_x;
    int target_grid_y = (target_y) / resolution + origin_grid_y;
    
    AStar::Generator generator;
    // Set 2d map size.
    int check_size = 2/resolution;
    generator.setWorldSize({grid_x_size , grid_y_size});

    // You can use a few heuristics : manhattan, euclidean or octagonal.
    generator.setHeuristic(AStar::Heuristic::euclidean);
    generator.setDiagonalMovement(true);

    generator.clearCollisions();
    cloud_in_ptr->clear();
    // pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in_ptr(new pcl::PointCloud<pcl::PointXYZI>); 
    // pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_visual_ptr(new pcl::PointCloud<pcl::PointXYZI>);    

    // pcl::ApproximateVoxelGrid<pcl::PointXYZI> avg_filter;
      
    //cut the goal pose in the grid map
    if(target_grid_y < 0)
        target_grid_y = 0;
    else if(target_grid_y > grid_y_size)
        target_grid_y = grid_y_size;    


    //add the collision 
    // int buf_size = nh_.param("grid_buf_size", 6);

    // Add map collision
    int buf_size = 6;
    for (int grid_x = start_grid_x - check_size; grid_x < start_grid_x + check_size; grid_x++) {
        for (int grid_y = start_grid_y - check_size; grid_y < start_grid_y + check_size; grid_y++) {
            if (map_sub.data[grid_y * grid_x_size + grid_x] > 80) {
                // generator.addCollision({grid_x, grid_y}, buf_size);
                
                    // for(int i = 0; i < buf_size; i++)
                    // {
                    //     for (int k = 0; k < buf_size; k++)
                    //     {
                            pcl::PointXYZI pointTmp;
                            pointTmp.x = grid_x * map_sub.info.resolution +
                                            map_sub.info.resolution / 2 +
                                            map_sub.info.origin.position.x;
                            pointTmp.y = grid_y * map_sub.info.resolution+
                                            map_sub.info.resolution / 2 +
                                            map_sub.info.origin.position.y;

                            cloud_in_ptr->points.push_back(pointTmp);
                    //     }
                    // }
            }
        }    
    }

    // Filter map
    avg_filter.setLeafSize(0.1, 0.1, 0.1); 
    avg_filter.setInputCloud(cloud_in_ptr);
    avg_filter.filter(*cloud_in_ptr);

    // Add map collision
    for(auto point:cloud_in_ptr->points){
        int grid_x = point.x / resolution + origin_grid_x;
        int grid_y = point.y / resolution + origin_grid_y;
        generator.addCollision({grid_x, grid_y}, buf_size);
    }
    
    // // Visual map collision
    // for(auto point:cloud_in_ptr->points){
    //     int grid_x = point.x / resolution + origin_grid_x;
    //     int grid_y = point.y / resolution + origin_grid_y;
        
    //     for(int i = 0; i < buf_size; i++)
    //     {
    //         for (int k = 0; k < buf_size; k++)
    //         {
    //             pcl::PointXYZI pointTmp;
    //             pointTmp.x = (grid_x+ i - buf_size / 2) * map_sub.info.resolution +
    //                             map_sub.info.resolution / 2 +
    //                             map_sub.info.origin.position.x;
    //             pointTmp.y = (grid_y+ k - buf_size / 2) * map_sub.info.resolution+
    //                             map_sub.info.resolution / 2 +
    //                             map_sub.info.origin.position.y;

    //             cloud_visual_ptr->points.push_back(pointTmp);
    //         }
    //     }

    // }

    // Add obstacle collision 
    for(auto point: obstacle.points)
    {
        Eigen::Vector4f local_cor;
        local_cor[0] = point.x;
        local_cor[1] = point.y;
        local_cor[2] = 0;
        local_cor[3] = 1;
        Eigen::Vector4f global_cor;
        global_cor = TF_matrix*local_cor; 
        int obstacle_grid_x  = global_cor[0] / resolution + origin_grid_x;
        int obstacle_grid_y  = global_cor[1] / resolution + origin_grid_y;

        generator.addCollision({obstacle_grid_x , obstacle_grid_y}, buf_size);

        // for(int i = 0; i < buf_size; i++)
        // {
        //     for (int k = 0; k < buf_size; k++)
        //     {

        //         pcl::PointXYZI pointTmp;
        //         pointTmp.x = (obstacle_grid_x + i - buf_size / 2) * map_sub.info.resolution +
        //                         map_sub.info.resolution / 2 +
        //                         map_sub.info.origin.position.x;
        //         pointTmp.y = (obstacle_grid_y + k - buf_size / 2) * map_sub.info.resolution +
        //                         map_sub.info.resolution / 2 +
        //                         map_sub.info.origin.position.y;
        //         cloud_in_ptr->points.push_back(pointTmp);
        //     }
        // }
        // std::cout << "obstacle : "<< obstacle_grid_x << ", " << obstacle_grid_y << std::endl; 

    }
    

    // //visualize the collision grid
    // sensor_msgs::PointCloud2 collisionCloudMsg;
    // pcl::toROSMsg(*cloud_in_ptr, collisionCloudMsg);
    // collisionCloudMsg.header.frame_id = "map";
    // collisionCloudMsg.header.stamp = ros::Time::now();
    // pubCollisionPoints.publish(collisionCloudMsg);

    //// FIND A STAR PATH /////

    auto _path = generator.findPath({start_grid_x, start_grid_y}, {target_grid_x, target_grid_y});

    std::cout << "start : "<< start_grid_x << ", " << start_grid_y << std::endl;
    std::cout << "target : "<< target_grid_x << ", " << target_grid_y << std::endl;
    std::cout << "path size: " << _path.size() << std::endl;
    std::cout << "wall size: " << generator.walls.size() << std::endl;

    // Publish a star path
    nav_msgs::Path AStartPathMsg;
    AStartPathMsg.header.stamp = ros::Time();
    AStartPathMsg.header.frame_id = "map";
    if (_path.size() <= 100){
        for(auto& coordinate : _path) {
            geometry_msgs::PoseStamped poseBuf;
            poseBuf.pose.position.x = (coordinate.x - origin_grid_x) * map_sub.info.resolution; 
            poseBuf.pose.position.y = (coordinate.y - origin_grid_y) * map_sub.info.resolution;
            AStartPathMsg.poses.push_back(poseBuf);

        }

    }
    pubAstarPath.publish(AStartPathMsg);
}

void AstarPlanner::CallbackOccupancyGrid(const nav_msgs::OccupancyGrid& msg)
{
    
    // if(!bNewGoalPose)
    //     return;

    // origin_x = msg.info.origin.position.x - 0.1;
    // origin_y = msg.info.origin.position.y - 0.1;
    // grid_x_size = msg.info.width;
    // grid_y_size = msg.info.height;
    origin_x = -10; origin_y = -10;
    grid_x_size = 400; grid_y_size = 400;
    resolution = msg.info.resolution;
    map_sub = msg;
    std::cout<<"ok 1"<<std::endl;
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
