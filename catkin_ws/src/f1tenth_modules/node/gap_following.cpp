/**
 * @file gap_following.cpp
 * @author Nathaniel Mallick (nmm109@pitt.edu)
 * @brief This file follows lab 4 of the F1Tenth lab modules (https://f1tenth.org/learn.html)
 * @version 0.1
 * @date 2022-07-29
 *
 * @copyright Copyright (c) 2022
 *
 */
#include <ros/ros.h>

#include <sensor_msgs/LaserScan.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <ackermann_msgs/AckermannDrive.h>
#include <std_msgs/Int32MultiArray.h>

#include <f1tenth_modules/f1tenthUtils.hh>

class GapFollowing
{
    private:
        ros::NodeHandle n;
        ros::Publisher drivePub;
        ros::Subscriber scanSub, muxSub;

        ackermann_msgs::AckermannDriveStamped drive;
        std::string driveTopic;

        lidarIntrinsics lidarData;
        pointScan closestPoint;

        int muxIdx;
        int scanStartIdx, scanEndIdx;
        double rb;
        bool enabled;

    public:

    GapFollowing():
        enabled(false),
        n(ros::NodeHandle("~"))
    {
        lidarData = getLidarInfoFromTopic(n, "/scan");
        if (!lidarData.valid)
            exit(-1);

        n.getParam("gap_follow_idx", muxIdx);
        n.getParam("gap_follow_topic", driveTopic);
        n.getParam("rb", rb);

        // pubs
        drivePub = n.advertise<ackermann_msgs::AckermannDriveStamped>(driveTopic, 1);

        // subs
        scanSub = n.subscribe("/scan", 1, &GapFollowing::scan_cb, this);
        muxSub = n.subscribe("mux", 1, &GapFollowing::mux_cb, this);

        scanStartIdx = getScanIdx((-M_PI/2.0), lidarData);
        scanEndIdx = getScanIdx((M_PI/2.0), lidarData);
    }

    void mux_cb(const std_msgs::Int32MultiArray &msg)
    {
        enabled = msg.data[muxIdx];
    }

    //
    // Do we want a copy or a reference of the LaserScan object?
    //
    void scan_cb(const sensor_msgs::LaserScan &msg)
    {
        auto min_point = std::make_pair(-1, msg.range_max);

        // Limit scans from -pi/2 -> pi/2
        for(size_t i = scanStartIdx; i <= scanEndIdx; i++)
        {
            if (msg.ranges[i] < min_point.second)
            {
                min_point.first = i;
                min_point.second = msg.ranges[i];
            }
        }

        if(min_point.first < 0)
            return;

        closestPoint =
            {min_point.second, min_point.first*msg.angle_increment+msg.angle_min, static_cast<size_t>(min_point.first)};

        // calculate start and end range of the bubble within the scan
        auto theta = std::acos(rb/closestPoint.dist);
        auto bubble_start_idx = getScanIdx(closestPoint.angle + theta, lidarData);
        auto bubble_end_idx = getScanIdx(closestPoint.angle - theta, lidarData);

        std::vector<size_t> zeros_indices{0};
        std::pair<size_t, size_t> max_sequence_indices;
        pointScan point;
        double r = rb;
        double max_sequence{0.0};

        // Check all points in the scan range of the bubble
        for(size_t i = bubble_start_idx; i <= bubble_end_idx; i++)
        {
            point.angle = i*msg.angle_increment + msg.angle_min;
            point.dist = msg.ranges[i];
            point.idx = i; //may be unecessary

            r = std::sqrt(
                std::pow(point.dist,2)
                + std::pow(closestPoint.dist,2)
                - 2*point.dist*closestPoint.dist*std::cos(point.angle - closestPoint.angle)
                );

            // Store the index of the 'zero'
            if( r < rb )
                zeros_indices.push_back(i);
        }

        //
        // Checking for the largest non-zero sequence
        //

        // from the start of the scan to the first indexed zero (in the buble)
        if(zeros_indices.front() > max_sequence)
        {
            max_sequence_indices.first = 0;
            max_sequence_indices.second = zeros_indices.front();
            max_sequence = zeros_indices.front();
        }

        // This should be thought out. If the angular
        // sweep of the bubble is greater than 50%(?) that
        // of the scan then this code is necessary
        for(size_t i = 0; i < zeros_indices.size(); i++)
        {
            if(zeros_indices[i] - zeros_indices[i-1] > max_sequence)
            {
                max_sequence_indices.first = zeros_indices[i-1];
                max_sequence_indices.second = zeros_indices[i];
                max_sequence = zeros_indices[i] - zeros_indices[i-1];
            }
        }

        // last indexed zero to the end of the scan
        if(scanEndIdx-scanStartIdx-zeros_indices.back() > max_sequence)
        {
            max_sequence_indices.first = zeros_indices.back(); //should this be back + 1?
            max_sequence_indices.second = scanEndIdx;
            max_sequence = scanEndIdx-scanStartIdx-zeros_indices.back();
        }

        // Find the largest point away from us within the max sequence
        auto max_point = std::make_pair(-1, msg.range_min);
        for(size_t i = max_sequence_indices.first; i < max_sequence_indices.second; i++)
        {
            if(msg.ranges[i] >= max_point.second)
            {
                max_point.first = i;
                max_point.second = msg.ranges[i];
            }
        }

        if(max_point.first < 0)
            return;

        // Set the steering angle to the farthest point
        // (TODO) Set this up to be a helper function with custom structs
        drive.header.stamp = ros::Time::now();
        drive.header.frame_id = "drive_gap_following";
        drive.drive.steering_angle = max_point.first*msg.angle_increment + msg.angle_min;
        drive.drive.steering_angle_velocity = 0.0;
        drive.drive.speed = 1.0;

        drivePub.publish(drive);
    }
};


int main(int argc, char **argv)
{

}

