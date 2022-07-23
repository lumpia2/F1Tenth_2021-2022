/**
 * @file wall_follow.cpp
 * @author Nathaniel Mallick (nmm109@pitt.edu)
 * @brief This file follows lab 3 of the F1Tenth lab modules 
 *          (https://f1tenth-coursekit.readthedocs.io/en/stable/assignments/labs/lab3.html)
 * @version 0.1
 * @date 2022-07-18
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include <ros/ros.h> 
#include <sensor_msgs/Image.h>
#include <sensor_msgs/LaserScan.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <ackermann_msgs/AckermannDrive.h>

#include <cmath>
#define pi M_PI // lazily avoiding uppercase variables for science 


class WallFollow 
{ 
    private: 
        ros::NodeHandle n; 
        ros::Publisher drive_pub; 
        ros::Subscriber scan_sub, mux_sub; 

        struct 
        {
            double kp, ki, kd; 
        } gains; 

        double err, prev_err; 
        double vel;
        double p,i,d; 
    public: 
        WallFollow(): 
            err(0.0), prev_err(0.0), 
            p(0.0), i(0.0), d(0.0), 
            gains({.kp=0.0, .ki=0.0, .kd=0.0}),
            n(ros::NodeHandle("~"))
            
        {
            // pubs

            // subs 
        } 

        void pid_control(const double &err, const double &vel);
        void lidar_cb(const sensor_msgs::LaserScan &msg); 
        double getRange(const sensor_msgs::LaserScan &data, const double &angle);
        double followLeft(); // need params 

   
        
}; 