# Global self-calibration for UWB
## Global self-calibration for UWB anchor positions with Lidar and UWB
An implementation for UWB anchor positions global self-calibration with Lidar using a `modified-A-LOAM` and UWB localization with a regularization method.


## 1. Prerequisites
### 1.1 **Ubuntu** and **ROS**
Ubuntu 64-bit 20.04.
ROS Noetic. [ROS Installation](http://wiki.ros.org/ROS/Installation)

### 1.2. **Ceres Solver**
Follow [Ceres Installation](http://ceres-solver.org/installation.html).

### 1.3. **PCL**
Follow [PCL Installation](https://pointclouds.org/downloads/#linux).


## 2. Build
Clone this repository and the `A-modified-A-LOAM` and catkin_make:

```
    cd ~/catkin_ws/src
    git clone https://github.com/L53317/global_selfcalibration.git

    git clone https://github.com/L53317/A-modified-A-LOAM.git

    cd ../
    catkin_make
    source ~/catkin_ws/devel/setup.bash
```

## 3. UWB self-calibration
The DATASET includes the UWB distance measurements, the anchor's initial positions and the corresponding Lidar cloud points. You can run the following commands to get optimal estimations of the UWB anchor's positions.
```
    roslaunch aloam_velodyne aloam_velodyne_HDL_32.launch
    rosrun global_selfcalibration global_selfcalibration_node
    rosbag play YOUR_DATASET_FOLDER/LIDAR_UWB.bag
```

## 4. TODO
Update the self-calibration implementation with respect to distance measurements.

## 5. Acknowledgements
Thanks for [VINS-Fusion](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion) and the [updated VINS-Fusion](https://github.com/L53317/VINS-Fusion).
For Licenses of the component codes, users are encouraged to read the material of these original projects.

