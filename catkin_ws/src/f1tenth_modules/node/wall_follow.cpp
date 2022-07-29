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

#include <std_msgs/Int32MultiArray.h>
#include <nav_msgs/Odometry.h>

#include <std_msgs/Int32MultiArray.h>
#include <nav_msgs/Odometry.h>

#include <cmath>
#include <f1tenth_modules/f1tenthUtils.hpp>

#define pi M_PI // lazily avoiding uppercase variables for science

/**
 * (todo)
 * - log P controller values
 * - need to test and tune PID controller
 */

class WallFollow
{
    private:
        ros::NodeHandle n;
        ros::Publisher drivePub;
        ros::Subscriber scanSub, muxSub;

        double dt = 1/60.0;
        ackermann_msgs::AckermannDriveStamped drive;
        std::string driveTopic;

        pidGains gains;
        lidarIntrinsics lidarData;

        int muxIdx;
        int aIdx, bIdx;

        double sp, prevErr, err;
        double p,i,d;
        double L;
        double theta = 70.0*pi/180.0; // [theta = 20 deg] (0 < theta < 70deg)

        bool enabled, done;

    public:
        WallFollow() = delete;
        WallFollow(double rate):
            dt(1/rate),
            enabled(false), done(false),
            prevErr(0.0),
            p(0.0), i(0.0), d(0.0),
            n(ros::NodeHandle("~"))
        {
            // Extract  lidar info from one message
            boost::shared_ptr<const sensor_msgs::LaserScan>
                tmpScan = ros::topic::waitForMessage<sensor_msgs::LaserScan>("/scan", n, ros::Duration(10.0));

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
            } else
            {
                ROS_INFO_ONCE("Couldn't extract lidar instrinsics... \nEXITING");
                exit(-1);
            }

            n.getParam("wall_follow_idx", muxIdx);
            n.getParam("wall_follow_topic", driveTopic);
            n.getParam("kp", gains.kp);
            n.getParam("ki", gains.ki);
            n.getParam("kd", gains.kd);
            n.getParam("sp", sp);

            ROS_INFO("");
            ROS_INFO("\tkp: %f\tki: %f\tkd: %f", gains.kp, gains.ki, gains.kd);
            ROS_INFO("");

            // pubs
            drivePub = n.advertise<ackermann_msgs::AckermannDriveStamped>(driveTopic, 1);

            // subs
            scanSub = n.subscribe("/scan", 1, &WallFollow::lidar_cb, this);
            muxSub = n.subscribe("/mux", 1, &WallFollow::mux_cb, this);

            // We want this index the angle thats orthogonally
            // to the left of the front of the car _|
            bIdx = (int)round((pi/2.0-lidarData.min_angle)/lidarData.scan_inc);
            aIdx = (int)round((((pi/2.0)-theta)-lidarData.min_angle)/lidarData.scan_inc);
            ROS_INFO("Scanning data at angles %f - %f",
                lidarData.min_angle + (lidarData.scan_inc*aIdx),
                lidarData.min_angle + (lidarData.scan_inc*bIdx));

            // Update theta to be MORE accurate due to rounding errors in finding our idx
            theta = lidarData.scan_inc*(bIdx - aIdx);
            ROS_INFO("Theta: %f", theta);

            drive.drive.speed=0.0;
        }


        void mux_cb(const std_msgs::Int32MultiArray &msg)
        {
            // Set the mux idx to verify wether to
            //  turn the PID controller on/off.
            enabled = msg.data[muxIdx];
            done = !enabled;

            if(enabled)
                ROS_INFO("PID node enabled.");
            else
                ROS_INFO("PID node disabled.");

            // (TODO) Maybe reset the PID values when "else"
        }

        void lidar_cb(const sensor_msgs::LaserScan &msg)
        {
            // auto pidStartTime = ros::Time::now();

            /////!!!!! NEED TO FILTER FOR BAD DISTANCES (inf &&&& <0)
            auto a = msg.ranges[aIdx];
            auto b = msg.ranges[bIdx];

            auto alpha = std::atan((a*std::cos(theta)-b)/(a*std::sin(theta)));
            auto dist_1 = (b*std::cos(alpha)) + (drive.drive.speed*dt)*std::sin(alpha);

            err = sp - dist_1;

            if(enabled)
                pid_control(err);

            prevErr = err;
        }

        void pid_control(const double &err)
        {
            p = err;
            i += err; // may need to be clamped
            d = (err-prevErr)/dt;

            const auto steer_angle = -(gains.kp*p + gains.ki*i + gains.kd*d);
            const auto steer_ang_deg = steer_angle*(180.0/pi);
            const auto abs_steer_ang_deg = std::abs(steer_ang_deg);

            //
            // TODO: Change these limits to compare against radians to minimize conversions
            //
            if(abs_steer_ang_deg >= 0.0 && abs_steer_ang_deg<10.0)
                drive.drive.speed = 1.5;
            else if(abs_steer_ang_deg>=10.0 && abs_steer_ang_deg<=20.0)
                drive.drive.speed = 1.0;
            else
                drive.drive.speed = 0.5;

            drive.header.stamp = ros::Time::now();
            drive.header.frame_id = "drive";
            drive.drive.steering_angle = steer_angle;
            drive.drive.steering_angle_velocity = 0.0;

            drivePub.publish(drive);
        }

        ros::Rate getRate() const
        {
            return 1/dt;
        }

        bool isDone() const
        {
            return done;
        }

        void spin()
        {
            ros::spinOnce();
        }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "wall_follow");
    WallFollow w(60.0);
    ros::Rate rate(w.getRate());

    while(!w.isDone())
    {
        w.spin();
        rate.sleep();
    }
}
