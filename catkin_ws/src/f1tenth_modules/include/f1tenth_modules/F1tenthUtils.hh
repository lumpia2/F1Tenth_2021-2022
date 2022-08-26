/**
 * @file f1tenthUtils.hpp
 * @author Nathaniel Mallick (nmm109@pitt.edu)
 * @brief Utility header file for the f1tenth_modules
 * @version 0.1
 * @date 2022-07-27
 *
 * @copyright Copyright (c) 2022
 */
#ifndef F1TENTH_UTILS_HH
#define F1TENTH_UTILS_HH

#include <cmath>
#include <string>
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Pose.h>

// #define pi M_PI // lazily avoiding uppercase variables for science

struct pidGains
{
    double kp;
    double ki;
    double kd;
};

struct lidarIntrinsics
{
    int num_scans;
    double min_angle;
    double max_angle;
    double scan_inc;
    bool valid;
};

struct pointScan
{
    double dist;
    double angle; //radians
    geometry_msgs::Point p;
};

struct rvizOpts
{
    uint32_t color;
    std::string frame_id;
    std::string ns;
    geometry_msgs::Pose pose;
    geometry_msgs::Vector3 scale;
    std::string topic;
};

struct carIntrinsics
{
    double width, wheelbase, base_link;
};

/**
 * @brief Extract lidar instrinsic information from specific topic
 *
 * @param n A ROS node to extract the message
 * @param topic The scan topic relative to the lidar scan subject
 * @return lidarIntrinsics Data extracted from the lide topic
 */
lidarIntrinsics getLidarInfoFromTopic(ros::NodeHandle &n , const std::string &topic);



/**
 * @brief Returns the index for a given angle in a list of lidar scan ranges
 *
 * @param theta The desired angle to find the corresponding index to
 * @param lidarData Lidar data object holding necessary info about each lidar scan
 * @return unsigned The closest index corresponding to the given angle
 *
 * @warning This function will return the index corresponding to the CLOSEST index.
 *          It may be necessary to recalculate the angle (theta) with the returned index.
 */
unsigned getScanIdx(const double &theta, const lidarIntrinsics &lidarData);


#endif