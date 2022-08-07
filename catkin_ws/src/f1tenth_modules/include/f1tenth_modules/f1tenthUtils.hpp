/**
 * @file f1tenthUtils.hpp
 * @author Nathaniel Mallick (nmm109@pitt.edu)
 * @brief Utility header file for the f1tenth_modules
 * @version 0.1
 * @date 2022-07-27
 *
 * @copyright Copyright (c) 2022
 */
#ifndef F1TENTH_UTILS_
#define F1TENTH_UTILS_

#include <cmath>

#define pi M_PI // lazily avoiding uppercase variables for science

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
    size_t idx;
};

lidarIntrinsics getLidarInfoFromTopic(ros::NodeHandle &n ,const std::string &topic)
{
    boost::shared_ptr<const sensor_msgs::LaserScan>
                tmpScan = ros::topic::waitForMessage<sensor_msgs::LaserScan>("/scan", n, ros::Duration(10.0));
    lidarIntrinsics lidarData;

        if(tmpScan != NULL)
        {
            lidarData.scan_inc = tmpScan->angle_increment;
            lidarData.min_angle = tmpScan->angle_min;
            lidarData.max_angle = tmpScan->angle_max;
            lidarData.num_scans =
                (int)ceil((lidarData.max_angle - lidarData.min_angle)/lidarData.scan_inc);

            ROS_INFO("");
            ROS_INFO("Min Angle:\t%f", lidarData.min_angle);
            ROS_INFO("Max Andgle:\t%f", lidarData.max_angle);
            ROS_INFO("Scan Incr:\t%f", lidarData.scan_inc);
            ROS_INFO("Num scans:\t%d", lidarData.num_scans);
            ROS_INFO("");
            lidarData.valid = true;
        } else
        {
            ROS_INFO_ONCE("Couldn't extract lidar instrinsics... ");
            lidarData.valid = false;
            return lidarData;
        }
    return lidarData;
}


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
unsigned getScanIdx(const double &theta, const lidarIntrinsics &lidarData)
{
    return (int)round((theta-lidarData.min_angle)/lidarData.scan_inc);
}

#endif