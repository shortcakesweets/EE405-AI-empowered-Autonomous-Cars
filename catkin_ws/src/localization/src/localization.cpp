#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
// #include <sensor_msgs/Imu.h>
#include <geometry_msgs/Pose.h>
#include <iostream>
#include <pcl/io/pcd_io.h>  
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/approximate_voxel_grid.h>
// #include <pcl/registration/ndt.h>
// #include <pcl/registration/gicp.h>
// #include <pcl/registration/icp_nl.h>
#include <laser_geometry/laser_geometry.h>
#include <tf/transform_broadcaster.h>
#include <fast_gicp/gicp/fast_gicp.hpp>
// #include <pcl/registration/correspondence_estimation.h>
// #include <pcl/filters/extract_indices.h>
// #include <pcl/filters/statistical_outlier_removal.h>
// #include <fstream>
// #include <fast_gicp/gicp/fast_vgicp_cuda.hpp>
// #include <pclomp/ndt_omp.h>
// #include <iostream>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> CloudT;
using namespace std;

/* Setup parameters for point cloud */
CloudT::Ptr cloud_in (new CloudT); //cloud input from laser
CloudT::Ptr cloud_target (new CloudT); //target input from map
CloudT::Ptr cloud_in_filter (new CloudT); // filter out cloud 
CloudT::Ptr cloud_target_filter (new CloudT); // filter out cloud 
CloudT::Ptr cloud_result (new CloudT); //final transformed cloud
CloudT::Ptr cloud_previous (new CloudT); //previous cloud input from laser
CloudT::Ptr cloud_obstacles (new CloudT); // cloud from laser use to check obstacle
CloudT::Ptr cloud_obstacles_visual (new CloudT);
pcl::PointIndices::Ptr inliers_input (new pcl::PointIndices());
pcl::PointIndices::Ptr inliers_target (new pcl::PointIndices());


laser_geometry::LaserProjection lp;

/* Registration for scan matching */
// pcl::Registration<PointT, PointT>::Ptr registra;
pcl::IterativeClosestPoint<PointT, PointT> icp;
// pcl::NormalDistributionsTransform<PointT, PointT> ndt;
// pcl::GeneralizedIterativeClosestPoint<PointT,PointT> gicp;
// pcl::IterativeClosestPointNonLinear<PointT, PointT> icp_nl;
// fast_gicp::FastGICP<PointT, PointT> f_gicp;


/* Filter for scan matching */
pcl::ApproximateVoxelGrid<PointT> avg_filter;
pcl::ApproximateVoxelGrid<PointT> avg_filter1;
// pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor_filter;

/* Paramter for publish odom and path */
sensor_msgs::PointCloud::Ptr cloud_project (new sensor_msgs::PointCloud); //project from laser
sensor_msgs::PointCloud::Ptr align_cloud (new sensor_msgs::PointCloud); //point cloud from map
sensor_msgs::PointCloud::Ptr filter_cloud (new sensor_msgs::PointCloud); //point cloud from map
Eigen::Matrix4f matrix = Eigen::Matrix4f::Identity(); // Estimated transformation matrix by register 
nav_msgs::Path::Ptr path (new nav_msgs::Path); // Obstable 

/* Initial guess position of car */
Eigen::Matrix4f init_guess = Eigen::Matrix4f::Identity();

/* Declare callback functions */
void callback_map(const nav_msgs::OccupancyGridConstPtr &msg);
void callback_scan(const sensor_msgs::LaserScan::ConstPtr &scan);

/* Setup for ROS */

ros::Publisher pub_path;


/* Main loop */
int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_ndt");
    ros::NodeHandle n;
    pub_path = n.advertise<nav_msgs::Path>("/obstacle",1);

    /* Declare Subscribers */
    ros::Subscriber sub_map = n.subscribe("/map", 1, callback_map);
    ros::Subscriber sub_laser = n.subscribe("/scan", 1, callback_scan);

    /* Declare Publisher */
    ros::Publisher pub_pose = n.advertise<nav_msgs::Odometry>("/odom",1);
    ros::Publisher align_pcl = n.advertise<sensor_msgs::PointCloud>("/map_pointcloud",1);
    ros::Publisher filter_pcl = n.advertise<sensor_msgs::PointCloud>("/obstacle_pointcloud",1);

    // // Setup for filter 
    avg_filter.setLeafSize(0.1, 0.1, 0.1);
    avg_filter1.setLeafSize(0.02, 0.02, 0.02);

    // Setup initial guess:
    init_guess(0,0) = 0;
    init_guess(0,1) = 1;
    init_guess(0,3) = -8;
    init_guess(1,0) = -1;
    init_guess(1,1) = 0;
    init_guess(1,3) = 1.7;

    while (ros::ok()){

        // Publish transform from map to laser
        static tf::TransformBroadcaster br;
        tf::Transform transform;
        tf::Vector3 translate(matrix(0,3), matrix(1,3), matrix(2,3));
        tf::Matrix3x3 rot_mat(  matrix(0,0), matrix(0,1), matrix(0,2),
                                matrix(1,0),matrix(1,1), matrix(1,2),
                                matrix(2,0),matrix(2,1), matrix(2,2));
        tf::Quaternion q;
        rot_mat.getRotation(q);
        transform.setOrigin(translate);
        transform.setRotation(q);
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "laser"));
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "imu_link"));

        // Publish estimated pose
        nav_msgs::Odometry odom; // estimated pose of car
        odom.header.frame_id = "map";
        odom.child_frame_id = "laser";
        odom.header.stamp = ros::Time::now();
        odom.pose.pose.position.x = matrix(0,3);
        odom.pose.pose.position.y = matrix(1,3);
        odom.pose.pose.position.z = matrix(2,3);
        odom.pose.pose.orientation.x = q.getX();
        odom.pose.pose.orientation.y = q.getY();
        odom.pose.pose.orientation.z = q.getZ();
        odom.pose.pose.orientation.w = q.getW();
        pub_pose.publish(odom); // publish pose

        // // Publish path
        // path->header.frame_id = "map";
        // path->header.stamp = ros::Time::now();
        // geometry_msgs::PoseStamped pose_stamp;
        // pose_stamp.pose.position.x = matrix(0,3);
        // pose_stamp.pose.position.y = matrix(1,3);
        // pose_stamp.pose.position.z = matrix(2,3);
        // pose_stamp.pose.orientation.x = q.getX();
        // pose_stamp.pose.orientation.y = q.getY();
        // pose_stamp.pose.orientation.z = q.getZ();
        // pose_stamp.pose.orientation.w = q.getW();
        // path->poses.push_back(pose_stamp);
        // pub_path.publish(path);

        // //Publish filter point cloud of map
        align_cloud->header.frame_id = "map";
        align_cloud->header.stamp = ros::Time::now();
        align_cloud->points.clear();
        for(auto& point: cloud_target->points){
            geometry_msgs::Point32 align_point;
            align_point.x = point.x;
            align_point.y = point.y;
            align_point.z = point.z;
            align_cloud->points.push_back(align_point);
        }
        align_pcl.publish(align_cloud);

        // Publish filter point cloud obstacle
        filter_cloud->header.frame_id = "laser";
        filter_cloud->header.stamp = ros::Time::now();
        filter_cloud->points.clear();
        for(auto& point: cloud_obstacles_visual->points){
            geometry_msgs::Point32 filter_point;
            filter_point.x = point.x;
            filter_point.y = point.y;
            filter_point.z = point.z;
            filter_cloud->points.push_back(filter_point);
        }
        filter_pcl.publish(filter_cloud);

        // // Save waypoints 
        // location_x = matrix(0,3);
        // location_y = matrix(1,3);
        // if (sqrt(pow(location_x-location_x_old,2) + pow(location_y-location_y_old,2)) >= 0.1) {
        //     myfile << location_x << " " << location_y << "\n";
        //     location_x_old = location_x; location_y_old = location_y;
        // }

        // Sleep
        // rate.sleep();
        ros::spinOnce();
    }
    // myfile.close();
    return 0;
}

/* Callback the map topic */
void callback_map(const nav_msgs::OccupancyGridConstPtr &msg) {
    
    int numel = msg->info.width * msg->info.height; // number of pixel on map
    float resolution = msg->info.resolution; // resolution of map
    float origin_x = msg->info.origin.position.x; // origin x of map 
    float origin_y = msg->info.origin.position.y; // origin y of map 

    // Update point cloud target
    for(int i=0; i<numel; i++){
        float height = i / msg->info.width; //pixel height
        float width = i % msg->info.width; //pixel width 
        if (msg->data[i] > 90)
        {
            PointT point;
            point.x = width*resolution + origin_x;
            point.y = height*resolution + origin_y;
            point.z = 0.0;
            cloud_target->points.push_back(point); // add point to point cloudtarget
        }
    }
}



void callback_scan(const sensor_msgs::LaserScan::ConstPtr& scan){
    
    // Update cloud previous
    *cloud_previous = *cloud_in;

    // Convert laser scan to input point cloud
    cloud_in->clear(); // reset cloud input
    cloud_obstacles->clear(); // reset cloud obstacle
    cloud_obstacles_visual->clear();

    path->poses.clear();
    lp.projectLaser(*scan, *cloud_project);

    if (cloud_project->points.size() != 0){
        for (auto& point: cloud_project->points){
            PointT output_point;
            output_point.x = point.x;
            output_point.y = point.y;
            output_point.z = 0;
            // cloud_in->points.push_back(output_point);

            // Limit range of point cloud
            if (sqrt(pow(point.x,2) + pow(point.y,2)) < 4)
            {
                cloud_in->points.push_back(output_point);
            }
        }
    }

    // Filter input point cloud 
    avg_filter1.setInputCloud(cloud_in);
    avg_filter1.filter(*cloud_in);

    // Filter cloud obstacle
    avg_filter.setInputCloud(cloud_in);
    avg_filter.filter(*cloud_obstacles);

    // Add obstacle to path
    if (cloud_obstacles->points.size() != 0){
        for (auto point : cloud_obstacles->points)
        {
            
            // if((sqrt(pow(point.x,2) + pow(point.y,2)) < 0.5) && point.x>=0)
            if (abs(point.y) <= 0.15 && point.x >= 0.1 && point.x <= 1.4) 
            {
                // path->header.frame_id = "map";
                // path->header.stamp = ros::Time::now();
                // geometry_msgs::PoseStamped pose_stamp;
                // pose_stamp.pose.position.x = point.x;
                // pose_stamp.pose.position.y = point.y;
                // pose_stamp.pose.position.z = 0;
                // pose_stamp.pose.orientation.x = 0;
                // pose_stamp.pose.orientation.y = 0;
                // pose_stamp.pose.orientation.z = 0;
                // pose_stamp.pose.orientation.w = 1;
                // path->poses.push_back(pose_stamp);
                // pub_path.publish(path);
                cloud_obstacles_visual->points.push_back(point);
            }
        }
    }

    cout << cloud_in->points.size() << ", " << cloud_target->points.size() << "," << cloud_obstacles_visual->points.size() <<endl;

    // Implement icp matching algorith
    if (cloud_previous->points.size()!=0 && cloud_in->points.size()!=0) {

        // Implement ICP 
        icp.setInputSource(cloud_in);
        icp.setInputTarget(cloud_target);
        icp.align(*cloud_result,init_guess);
        matrix = icp.getFinalTransformation();
        init_guess = matrix;

    }       
}



