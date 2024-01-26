
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
    void inputOdom(double t, Eigen::Vector3d OdomP, Eigen::Quaterniond OdomQ);
    void getGlobalOdom(Eigen::Vector3d &odomP, Eigen::Quaterniond &odomQ);
    nav_msgs::Path global_path;

private:
    void optimize();
    void updateGlobalPath();

    // format t, tx,ty,tz,qw,qx,qy,qz
    map<double, vector<double>> localPoseMap;
    map<double, vector<double>> globalPoseMap;
    map<double, vector<double>> UWBPositionMap;
    bool newUWB;
    std::mutex mPoseMap;
    Eigen::Matrix4d WUWB_T_WVIO;
    Eigen::Vector3d lastP;
    Eigen::Quaterniond lastQ;
    std::thread threadOpt;

};