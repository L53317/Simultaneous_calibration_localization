
#include "ros/ros.h"
#include "globalOptimization.h"
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <iostream>
#include <stdio.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <fstream>
#include <queue>
#include <mutex>
#include <pcl_conversions/pcl_conversions.h>
// #include <pcl/point_cloud.h>
// #include <pcl/point_types.h>

/* Inlcude a UWB message building for UWB data.
   To use official message, maybe replace it with a path having two (or more) poses.
 */

#include "uwb_localization_dwm/UWBrange.h"

GlobalOptimization globalEstimator;
ros::Publisher pub_global_odometry, pub_global_path, pub_car, pub_uwb_anchors, pub_uwb_map;
nav_msgs::Path *global_path;
double last_vio_t = -1;
// int anchors_number = 4;
std::mutex m_buf;
std::queue<geometry_msgs::PoseStamped::ConstPtr> uwbQueue; // For UWB; geometry_msgs::Vector3StampedConstPtr
std::queue<uwb_localization_dwm::UWBrange::ConstPtr> uwbRangeQueue;

void publish_car_model(double t, Eigen::Vector3d t_w_car, Eigen::Quaterniond q_w_car)
{
    visualization_msgs::MarkerArray markerArray_msg;
    visualization_msgs::Marker car_mesh;
    car_mesh.header.stamp = ros::Time(t);
    car_mesh.header.frame_id = "map";
    car_mesh.type = visualization_msgs::Marker::MESH_RESOURCE;
    car_mesh.action = visualization_msgs::Marker::ADD;
    car_mesh.id = 0;

    car_mesh.mesh_resource = "package://global_loam/models/car.dae";

    Eigen::Matrix3d rot;
    rot << 0, 0, -1, 0, -1, 0, -1, 0, 0;

    Eigen::Quaterniond Q;
    Q = q_w_car * rot; 
    car_mesh.pose.position.x    = t_w_car.x();
    car_mesh.pose.position.y    = t_w_car.y();
    car_mesh.pose.position.z    = t_w_car.z();
    car_mesh.pose.orientation.w = Q.w();
    car_mesh.pose.orientation.x = Q.x();
    car_mesh.pose.orientation.y = Q.y();
    car_mesh.pose.orientation.z = Q.z();

    car_mesh.color.a = 1.0;
    car_mesh.color.r = 1.0;
    car_mesh.color.g = 0.0;
    car_mesh.color.b = 0.0;

    float major_scale = 2.0;

    car_mesh.scale.x = major_scale;
    car_mesh.scale.y = major_scale;
    car_mesh.scale.z = major_scale;
    markerArray_msg.markers.push_back(car_mesh);
    pub_car.publish(markerArray_msg);
}

/* For UWB localization */
void UWB_callback(const geometry_msgs::PoseStamped::ConstPtr &UWB_msg)
{
    m_buf.lock();
    uwbQueue.push(UWB_msg);
    m_buf.unlock();
}

void UWBrange_callback(const uwb_localization_dwm::UWBrange::ConstPtr &UWBrange_msg)
{
    m_buf.lock();
    // double t, x, y, z, dis;
    // t = UWBrange_msg->header.stamp.toSec();
    double x, y, z;
    x = UWBrange_msg->anchor_position.x;
    y = UWBrange_msg->anchor_position.y;
    z = UWBrange_msg->anchor_position.z;
    // dis = UWBrange_msg->distance_to_tag;
    // printf("UWBrange_msg t : (x, y, z):    %f : (%f, %f, %f), distance_to_tag: %f \n",
    //        t, x, y, z, dis);
    uwbRangeQueue.push(UWBrange_msg);

    /* Get latest anchors' positions and publish */
    Eigen:: Vector3d global_t;
    Eigen:: Quaterniond global_q;
    Eigen::MatrixXd global_uwb_anchors;
    globalEstimator.getUWBanchors(global_t, global_q, global_uwb_anchors);

    nav_msgs::Odometry uwb_anchor_odom;
    uwb_anchor_odom.header = UWBrange_msg->header;
    uwb_anchor_odom.header.frame_id = "map";
    uwb_anchor_odom.child_frame_id = "map";
    /* UWB frame to Lidar frame */
    uwb_anchor_odom.pose.pose.position.x = global_t.x();
    uwb_anchor_odom.pose.pose.position.y = global_t.y();
    uwb_anchor_odom.pose.pose.position.z = global_t.z();
    uwb_anchor_odom.pose.pose.orientation.x = global_q.x();
    uwb_anchor_odom.pose.pose.orientation.y = global_q.y();
    uwb_anchor_odom.pose.pose.orientation.z = global_q.z();
    uwb_anchor_odom.pose.pose.orientation.w = global_q.w();
    /* UWB anchors initial positions */
    uwb_anchor_odom.twist.twist.linear.x = x;
    uwb_anchor_odom.twist.twist.linear.y = y;
    uwb_anchor_odom.twist.twist.linear.z = z;

    /* Coresponding UWB anchors initial positions, must already in last_UWB_anchorsPs map */
    int anchor_i = 0;
    int anchor_initial_pos_dimension = 3;
    bool find_anchor = false;
    for (anchor_i = 0; anchor_i < global_uwb_anchors.cols(); anchor_i++)
    {
        if (norm(x - global_uwb_anchors(0,anchor_i))<0.01 &&
            norm(y - global_uwb_anchors(1,anchor_i))<0.01 &&
            norm(z - global_uwb_anchors(2,anchor_i))<0.01)
            {
                find_anchor =true;
                break;
            }
    }

    if (find_anchor && global_uwb_anchors.cols() !=0) // No need !=0.
    {
        // with respect anchors index
        uwb_anchor_odom.twist.twist.angular.x = global_uwb_anchors(anchor_initial_pos_dimension+0,anchor_i);
        uwb_anchor_odom.twist.twist.angular.y = global_uwb_anchors(anchor_initial_pos_dimension+1,anchor_i);
        uwb_anchor_odom.twist.twist.angular.z = global_uwb_anchors(anchor_initial_pos_dimension+2,anchor_i);
        pub_uwb_anchors.publish(uwb_anchor_odom);

        sensor_msgs::PointCloud2 map_msg;
        pcl::PointCloud<pcl::PointXYZ>::Ptr uwb_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        for (int i = 0; i < global_uwb_anchors.cols(); i++)
        {
            pcl::PointXYZ point(global_uwb_anchors(anchor_initial_pos_dimension+0, i),
                                global_uwb_anchors(anchor_initial_pos_dimension+1, i),
                                global_uwb_anchors(anchor_initial_pos_dimension+2, i));
            // cout << "Anchor map: " << global_uwb_anchors(anchor_initial_pos_dimension+0, i) << " "
            //                        << global_uwb_anchors(anchor_initial_pos_dimension+1, i) << " "
            //                        << global_uwb_anchors(anchor_initial_pos_dimension+2, i) << endl;
            uwb_cloud->push_back(point);
        }

        pcl::toROSMsg(*uwb_cloud, map_msg);
        map_msg.header = uwb_anchor_odom.header;
        map_msg.header.frame_id = uwb_anchor_odom.header.frame_id;
        pub_uwb_map.publish(map_msg);
    }

    m_buf.unlock();

    // // write result to file
    // std::ofstream foutC("/home/liu/Downloads/output/uwb_map.csv", ios::app);
    // foutC.setf(ios::fixed, ios::floatfield);
    // foutC.precision(0);
    // foutC << pose_msg->header.stamp.toSec() * 1e9 << ",";
    // foutC.precision(5);
    // foutC << global_t.x() << ","
    //       << global_t.y() << ","
    //       << global_t.z() << ","
    //       << global_q.w() << ","
    //       << global_q.x() << ","
    //       << global_q.y() << ","
    //       << global_q.z() << ","
    //       << uwb_anchor_odom.twist.twist.linear.x() << ","
    //       << uwb_anchor_odom.twist.twist.linear.y() << ","
    //       << uwb_anchor_odom.twist.twist.linear.z() << ","
    //       << uwb_anchor_odom.twist.twist.angular.x() << ","
    //       << uwb_anchor_odom.twist.twist.angular.y() << ","
    //       << uwb_anchor_odom.twist.twist.angular.z()
    //       << endl;
    // foutC.close();
}

void vio_callback(const nav_msgs::Odometry::ConstPtr &pose_msg)
{
    // printf("vio_callback! \n");
    double t = pose_msg->header.stamp.toSec();
    last_vio_t = t;
    Eigen::Vector3d vio_t(pose_msg->pose.pose.position.x, pose_msg->pose.pose.position.y, pose_msg->pose.pose.position.z);
    Eigen::Quaterniond vio_q;
    vio_q.w() = pose_msg->pose.pose.orientation.w;
    vio_q.x() = pose_msg->pose.pose.orientation.x;
    vio_q.y() = pose_msg->pose.pose.orientation.y;
    vio_q.z() = pose_msg->pose.pose.orientation.z;
    globalEstimator.inputOdom(t, vio_t, vio_q);

    /* For UWB localization */
    m_buf.lock();
    while(!uwbQueue.empty())
    {
        geometry_msgs::PoseStamped::ConstPtr UWB_msg = uwbQueue.front();
        double uwb_t = UWB_msg->header.stamp.toSec();
        // printf("vio t: %f, uwb t: %f \n", t, uwb_t);
        // 100ms sync tolerance, sensor time synchronize
         /* Search all UWB pose in this interval.
            Note, with '[]' operation, only the newest item will be added (C++, Map, []). */
        if(uwb_t >= t - 0.1 && uwb_t <= t + 0.1)
        {
            // printf("receive UWB with timestamp %f\n", UWB_msg->header.stamp.toSec());
            double x = UWB_msg->pose.position.x;
            double y = UWB_msg->pose.position.y;
            double z = UWB_msg->pose.position.z;
            double pos_accuracy = 0; // UWB_msg->position_covariance[0];
            if(pos_accuracy <= 0)
                pos_accuracy = 0.8; // 1
            // printf("receive covariance %lf \n", pos_accuracy);
            // if(UWB_msg->status.status > 8)
                globalEstimator.inputUWB(t, x, y, z, pos_accuracy); // Add UWB poses with vio t
            uwbQueue.pop();
            break;
        }
        else if(uwb_t < t - 0.1)
            uwbQueue.pop();
        else if(uwb_t > t + 0.1)
            break;
    }
    // m_buf.unlock();

    // m_buf.lock();
    while(!uwbRangeQueue.empty())
    {
        uwb_localization_dwm::UWBrange::ConstPtr UWBRange_msg = uwbRangeQueue.front();
        double uwbrange_t = UWBRange_msg->header.stamp.toSec();
         /* Search all UWB distance in this interval.
            Note, only the newest will be added (C++, Map, []). */
        if(uwbrange_t >= t - 0.1 && uwbrange_t <= t + 0.1)
        {
            double x = UWBRange_msg->anchor_position.x;
            double y = UWBRange_msg->anchor_position.y;
            double z = UWBRange_msg->anchor_position.z;
            double dis = UWBRange_msg->distance_to_tag;
            double dis_accuracy = 0; // UWBRange_msg->position_covariance[0];
            if(dis_accuracy <= 0)
                {dis_accuracy = 0.95;}
            globalEstimator.inputUWBdistance(t, x, y, z, dis, dis_accuracy);//, &anchor_index

            uwbRangeQueue.pop();
            break;
        }
        else if(uwbrange_t < t - 0.1)
            uwbRangeQueue.pop();
        else if(uwbrange_t > t + 0.1)
            break;
    }
    m_buf.unlock();

    Eigen::Vector3d global_t;
    Eigen:: Quaterniond global_q;
    globalEstimator.getGlobalOdom(global_t, global_q);

    nav_msgs::Odometry odometry;
    odometry.header = pose_msg->header;
    odometry.header.frame_id = "map";
    odometry.child_frame_id = "map";
    odometry.pose.pose.position.x = global_t.x();
    odometry.pose.pose.position.y = global_t.y();
    odometry.pose.pose.position.z = global_t.z();
    odometry.pose.pose.orientation.x = global_q.x();
    odometry.pose.pose.orientation.y = global_q.y();
    odometry.pose.pose.orientation.z = global_q.z();
    odometry.pose.pose.orientation.w = global_q.w();
    pub_global_odometry.publish(odometry);
    pub_global_path.publish(*global_path);
    publish_car_model(t, global_t, global_q);

    cout << " Global pose(x,y,z, w,x,y,z): " << odometry.header.stamp << ", "
         << odometry.pose.pose.position.x << ", "
         << odometry.pose.pose.position.y << ", "
         << odometry.pose.pose.position.z << ",   "
         << odometry.pose.pose.orientation.x << ", "
         << odometry.pose.pose.orientation.y << ", "
         << odometry.pose.pose.orientation.z << ", "
         << odometry.pose.pose.orientation.w
         << endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "globalEstimator"); // scal
    ros::NodeHandle n("~");

    global_path = &globalEstimator.global_path;

    ros::Subscriber sub_UWB = n.subscribe("/uwb/localization/tag/hr_position", 100, UWB_callback); // /dwm1001/tag/tag/position
    // for (int i = 0; i < 4; i++)
    // {
    //     string uwb_range_topic = "/dwm1001/tag/tag/to/anchor/AN"+to_string(i)+"/distance";
    // }
    // ros::Subscriber sub_UWBRange0 = n.subscribe("/dwm1001/tag/tag/to/anchor/AN0/distance", 100, boost::bind(&UWBrange_callback, _1, "AN0"));
    ros::Subscriber sub_UWBRange0 = n.subscribe("/dwm1001/tag/tag/to/anchor/AN0/distance", 100, UWBrange_callback);
    ros::Subscriber sub_UWBRange1 = n.subscribe("/dwm1001/tag/tag/to/anchor/AN1/distance", 100, UWBrange_callback);
    ros::Subscriber sub_UWBRange2 = n.subscribe("/dwm1001/tag/tag/to/anchor/AN2/distance", 100, UWBrange_callback);
    ros::Subscriber sub_UWBRange3 = n.subscribe("/dwm1001/tag/tag/to/anchor/AN3/distance", 100, UWBrange_callback);
    // ros::Subscriber sub_vio = n.subscribe("/vins_estimator/odometry", 100, vio_callback);
    ros::Subscriber sub_vio = n.subscribe("/aft_mapped_to_init_high_frec", 100, vio_callback);
    pub_global_path = n.advertise<nav_msgs::Path>("uwb_global_path", 100);
    pub_global_odometry = n.advertise<nav_msgs::Odometry>("uwb_global_odometry", 100);
    pub_uwb_anchors = n.advertise<nav_msgs::Odometry>("uwb_map_anchors_position", 100);
    pub_uwb_map = n.advertise<sensor_msgs::PointCloud2>("uwb_map_pointcloud", 100);
    pub_car = n.advertise<visualization_msgs::MarkerArray>("lidar_car_model", 1000);

    ros::spin();
    return 0;
}
