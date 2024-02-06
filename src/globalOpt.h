
#pragma once
#include <vector>
#include <map>
#include <iostream>
#include <mutex>
#include <thread>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <ceres/ceres.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include "tic_toc.h"

using namespace std;

class GlobalOptimization
{
public:
    GlobalOptimization();
    ~GlobalOptimization();
    void inputUWB(double t, double x, double y, double z, double posAccuracy);
    void inputUWBdistance(double t, double x, double y, double z, double distance, double disAccuracy);
    void inputOdom(double t, Eigen::Vector3d OdomP, Eigen::Quaterniond OdomQ);
    void getGlobalOdom(Eigen::Vector3d &odomP, Eigen::Quaterniond &odomQ);
    // void getUWBanchors(Eigen::Vector3d &odomP, Eigen::Quaterniond &odomQ, Eigen::Vector3d (&anchorP)[4]);
    void getUWBanchors(Eigen::Vector3d &odomP, Eigen::Quaterniond &odomQ, Eigen::MatrixXd &anchorP);
    nav_msgs::Path global_path;
    // /* UWB anchor map store in poses of a path:
    //    pose.position: anchor estimate position;
    //    pose.orientation: anchor initial position.
    //  */
    // nav_msgs::Path UWB_anchor_Map;

private:
    void optimize();
    void updateGlobalPath();
    void updateAnchorMap();

    // format t, tx,ty,tz,qw,qx,qy,qz
    map<double, vector<double>> localPoseMap; // vio poses
    map<double, vector<double>> globalPoseMap; // optimization poses
    map<double, vector<double>> UWBPositionMap; // UWB poses

    // map<double, vector<double>> UWBDistanceMap; // should have 4 (anchor number) map, or 4 columns
    map<double, vector<double>> globalAnchorMap; // Anchor map with distances

    bool newUWB;
    bool newUWBdistance;
    std::mutex mPoseMap;
    Eigen::Matrix4d WUWB_T_WVIO;
    Eigen::Vector3d lastP;
    Eigen::Quaterniond lastQ;
    std::thread threadOpt;
    // int anchors_number = 4; // 2*4 (initial + optimization) or Vector6d
    // Eigen::Vector3d last_UWB_anchorsP[4]; // Add for 4 UWB anchors self-calibration. (Fixed order of total 4 anchors)
    // anchor map; row * col = (initial + optimization position = 6) * anchor_number
    Eigen::MatrixXd last_UWB_anchorsPs;//(6,4);
};