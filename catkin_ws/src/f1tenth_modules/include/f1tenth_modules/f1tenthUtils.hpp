/**
 * @file f1tenthUtils.hpp
 * @author Nathaniel Mallick (nmm109@pitt.edu)
 * @brief Utility header file for the f1tenth_modules
 * @version 0.1
 * @date 2022-07-27
 *
 * @copyright Copyright (c) 2022
 */

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
};