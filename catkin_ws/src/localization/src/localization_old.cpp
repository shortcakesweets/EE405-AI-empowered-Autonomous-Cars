#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Pose.h>
#include <iostream>
#include <pcl/io/pcd_io.h>  
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/registration/ndt.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/icp_nl.h>
#include <laser_geometry/laser_geometry.h>
#include <tf/transform_broadcaster.h>
#include <fast_gicp/gicp/fast_gicp.hpp>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <fstream>
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
pcl::PointIndices::Ptr inliers_input (new pcl::PointIndices());
pcl::PointIndices::Ptr inliers_target (new pcl::PointIndices());


laser_geometry::LaserProjection lp;

/* Registration for scan matching */
pcl::Registration<PointT, PointT>::Ptr registra;
pcl::IterativeClosestPoint<PointT, PointT> icp;
pcl::NormalDistributionsTransform<PointT, PointT> ndt;
pcl::GeneralizedIterativeClosestPoint<PointT,PointT> gicp;
pcl::IterativeClosestPointNonLinear<PointT, PointT> icp_nl;
fast_gicp::FastGICP<PointT, PointT> f_gicp;
// fast_gicp::FastVGICPCuda<pcl::PointXYZ, pcl::PointXYZ> vgicp_cuda;


/* Filter for scan matching */
pcl::ApproximateVoxelGrid<PointT> avg_filter;
pcl::ApproximateVoxelGrid<PointT> avg_filter1;
pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor_filter;

/* Paramter for publish odom and path */
sensor_msgs::PointCloud::Ptr cloud_project (new sensor_msgs::PointCloud); //project from laser
sensor_msgs::PointCloud::Ptr align_cloud (new sensor_msgs::PointCloud); //point cloud from map
sensor_msgs::PointCloud::Ptr filter_cloud (new sensor_msgs::PointCloud); //point cloud from map
Eigen::Matrix4f matrix = Eigen::Matrix4f::Identity(); // Estimated transformation matrix by register 
nav_msgs::Path::Ptr path (new nav_msgs::Path); // Estimated path 

/* Initial guess position of car */
Eigen::Matrix4f init_guess = Eigen::Matrix4f::Identity();
ros::Time current_time;
ros::Time previous_time;
geometry_msgs::Vector3 linear_acceleration;
geometry_msgs::Vector3 linear_acceleration_old;
geometry_msgs::Vector3 angular_velocity;
geometry_msgs::Vector3 angular_velocity_old;
double velocity_x = 0.0;
double velocity_y = 0.0;

/* Waypoints */
double location_x =0, location_y=0;
double location_x_old =0, location_y_old =0;

/* Declare callback functions */
void callback_map(const nav_msgs::OccupancyGridConstPtr &msg);
void callback_scan(const sensor_msgs::LaserScan::ConstPtr &scan);

/* Main loop */
int main(int argc, char **argv)
{
    /* Setup for ROS */
    ros::init(argc, argv, "test_ndt");
    ros::NodeHandle n;
    ros::Rate rate(30);
    ros::Time currentT = ros::Time::now();
    ros::Time previousT = ros::Time::now();

    
    /* Declare Subscribers */
    ros::Subscriber sub_map = n.subscribe("/map", 1, callback_map);
    ros::Subscriber sub_laser = n.subscribe("/scan", 1, callback_scan);

    /* Declare Publisher */
    ros::Publisher pub_pose = n.advertise<nav_msgs::Odometry>("/odom",1);
    ros::Publisher pub_path = n.advertise<nav_msgs::Path>("/path",1);
    ros::Publisher align_pcl = n.advertise<sensor_msgs::PointCloud>("/map_pointcloud",1);
    ros::Publisher filter_pcl = n.advertise<sensor_msgs::PointCloud>("/filter_pointcloud",1);

    // // Setup for filter 
    avg_filter.setLeafSize(0.08, 0.08, 0.08);
    avg_filter1.setLeafSize(0.02, 0.02, 0.02);
    // avg_filter.setDownsampleAllData(false);
    // sor_filter.setMeanK(20);
    // sor_filter.setStddevMulThresh(1.0);

    // // Setup Fast_GICP
    // gicp.setMaxCorrespondenceDistance(2);
    // gicp.setTransformationEpsilon(0.03);
    // gicp.setMaximumIterations(64);

    // // Setup ICP
    // // icp.setTransformationEpsilon(0.01);
    // // icp.setMaximumIterations(30);
    // // icp.setMaxCorrespondenceDistance(1);

    // Setup FATS-GICP
    // f_gicp.setMaxCorrespondenceDistance(1);
    // f_gicp.setNumThreads(4);
    // f_gicp.setMaximumIterations(30);
    // f_gicp.setTransformationEpsilon(0.01);
    //f_gicp.setSearchMethodSource(tree);

    // // // Setup for save waypoints
    // ofstream myfile;
    // myfile.open("waypoint.csv");
    // myfile.clear();
    // // myfile << "x(m) y(m) \n"; 
    // myfile <<"0 0 \n";

    // Setup time 
    current_time = ros::Time::now();
    previous_time = ros::Time::now();


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
        odom.twist.twist.linear.x = velocity_x;
        odom.twist.twist.linear.y = velocity_y;
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

        // Publish filter point cloud
        filter_cloud->header.frame_id = "laser";
        filter_cloud->header.stamp = ros::Time::now();
        filter_cloud->points.clear();
        for(auto& point: cloud_in->points){
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

    // // Filter the target point cloud
    // avg_filter.setInputCloud(cloud_target);
    // avg_filter.filter(*cloud_target);
}



void callback_scan(const sensor_msgs::LaserScan::ConstPtr& scan){
    
    // Update cloud previous
    *cloud_previous = *cloud_in;
    previous_time = current_time;
    current_time = ros::Time::now();

    // Convert laser scan to input point cloud
    cloud_in->clear(); // reset cloud input
    lp.projectLaser(*scan, *cloud_project);
    if (cloud_project->points.size() != 0){
        for (auto& point: cloud_project->points){
            PointT output_point;
            output_point.x = point.x;
            output_point.y = point.y;
            output_point.z = 0;
            // cloud_in->points.push_back(output_point);

            // Limit range of point cloud
            if (sqrt(pow(point.x,2) + pow(point.y,2)) < 4){
                cloud_in->points.push_back(output_point);
            }
          
        }
    }

    // Filter input point cloud 
    avg_filter1.setInputCloud(cloud_in);
    avg_filter1.filter(*cloud_in);

    // sor_filter.setInputCloud(cloud_in);
    // sor_filter.filter(*cloud_in);
    cout << cloud_in->points.size() << ", " << cloud_target->points.size() <<endl;


    // Implement icp matching algorith
    if (cloud_previous->points.size()!=0 && cloud_in->points.size()!=0) {

        // // Use ICP to predict initial initial guess for car location 
        // icp.setInputSource(cloud_in);
        // icp.setInputTarget(cloud_previous);
        // icp.align(*cloud_result);
        // init_guess = init_guess*icp.getFinalTransformation();

        // // Calculate the velocity of car
        // double dt = (current_time-previous_time).toSec();
        // velocity_x = icp.getFinalTransformation()(0,3)/dt;
        // velocity_y = icp.getFinalTransformation()(1,3)/dt;
        // cout << velocity_x<< ',' << velocity_y << ',' << dt << endl;

        // Implement Fast-Gicp to find real location of car 
        // f_gicp.clearSource();
        // f_gicp.clearTarget();
        // f_gicp.setInputSource(cloud_in);
        // f_gicp.setInputTarget(cloud_target);
        // f_gicp.align(*cloud_result, init_guess);
        // if (f_gicp.hasConverged()){
        //     matrix = f_gicp.getFinalTransformation(); 
        //     init_guess = matrix;
        // }

        // Implement FAST_GICP 
        icp.setInputSource(cloud_in);
        icp.setInputTarget(cloud_target);
        icp.align(*cloud_result,init_guess);
        matrix = icp.getFinalTransformation();
        init_guess = matrix;

    }       
}



