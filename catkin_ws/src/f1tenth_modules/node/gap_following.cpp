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

#include <f1tenth_modules/RvizWrapper.hh>
#include <f1tenth_modules/F1tenthUtils.hh>

class GapFollowing
{
    private:
        ros::NodeHandle n;
        ros::Publisher drivePub;
        ros::Subscriber scanSub, muxSub;

        ackermann_msgs::AckermannDriveStamped drive;
        std::string driveTopic;

        lidarIntrinsics lidarData;
        pointScan closestPoint,
                  furthestPoint;

        int muxIdx;
        int scanStartIdx, scanEndIdx;
        double rb;
        bool enabled;

        std::unique_ptr<RvizPoint> bubble;
        std::unique_ptr<RvizPoint> cp, fp;

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
        muxSub = n.subscribe("/mux", 1, &GapFollowing::mux_cb, this);

        scanStartIdx = getScanIdx((-M_PI/2.0), lidarData);
        scanEndIdx = getScanIdx((M_PI/2.0), lidarData);

        ROS_INFO("");
        ROS_INFO("Start index of scan : %d", scanStartIdx);
        ROS_INFO("End index of scan: %d", scanEndIdx);
        ROS_INFO("");

        // Rviz configuration
        geometry_msgs::Pose pose;
        geometry_msgs::Vector3 scale;

        pose.orientation.w = 1.0;
        scale.x = scale.y = 0.1;

        rvizOpts opts = {
                            .color=0xff0000,
                            .frame_id="laser",
                            .ns="point",
                            .pose=pose,
                            .scale=scale,
                            .topic="/dynamic_viz"
                        };

        // These points will be blue
        opts.color=0x00ff00;
        fp = std::make_unique<RvizPoint>(n, opts);
        cp = std::make_unique<RvizPoint>(n, opts);

        opts.color=0xff0000;
        bubble = std::make_unique<RvizPoint>(n, opts);

        cp->addTransformPair("base_link", "laser");
        fp->addTransformPair("base_link", "laser");
        bubble->addTransformPair("base_link", "laser");
    }

    void mux_cb(const std_msgs::Int32MultiArray &msg)
    {
        enabled = msg.data[muxIdx];
    }

    void scan_cb(const sensor_msgs::LaserScan &msg)
    {
        std::vector<geometry_msgs::Point> bubble_point_vector;
        std::pair<size_t, size_t> max_sequence_indices;
        std::vector<size_t> zeros_indices;

        auto scan_cp = msg.ranges;

        pointScan point_scan;
        geometry_msgs::Point point;

        point.z = 0.0;

        double r = rb;
        double max_sequence{0.0};
        auto min_point = std::make_pair(-1, msg.range_max);
        unsigned bubble_start_idx, bubble_end_idx;

        for (size_t i = scanStartIdx; i <= scanEndIdx; i++)
        {
            if (scan_cp[i] < min_point.second)
            {
                min_point.first = i;
                min_point.second = scan_cp[i];
            }
        }

        if (min_point.first < 0)
            return;

        closestPoint = {
                .dist = min_point.second,
                .angle = min_point.first*msg.angle_increment+msg.angle_min,
            };

        closestPoint.p.x = closestPoint.dist*std::cos(closestPoint.angle);
        closestPoint.p.y = closestPoint.dist*std::sin(closestPoint.angle);
        closestPoint.p.z = 0.0;

        cp->addTranslation(closestPoint.p);

        // calculate start and end range of the bubble within the scan
        if (closestPoint.dist < rb)
        {
            bubble_start_idx = scanStartIdx;
            bubble_end_idx = scanEndIdx;
        } else
        {
            auto theta = std::asin(rb/closestPoint.dist);
            bubble_start_idx = getScanIdx(closestPoint.angle - theta, lidarData);
            bubble_end_idx = getScanIdx(closestPoint.angle + theta, lidarData);
        }


        // Check all points in the scan range of the bubble
        for (size_t i = bubble_start_idx; i <= bubble_end_idx; i++)
        {
            if (i < scanStartIdx || i > scanEndIdx)
                continue;

            point_scan.angle = i*msg.angle_increment + msg.angle_min;
            point_scan.dist = scan_cp[i];

            r = std::sqrt(
                std::pow(point_scan.dist,2)
                + std::pow(closestPoint.dist,2)
                - 2*point_scan.dist*closestPoint.dist*std::cos(point_scan.angle - closestPoint.angle)
                );

            //
            // TODO(nmm): Add some sort of configuration to turn
            //  the rviz functionality on and off
            //

            // Store the index of the 'zero' and add the points
            // rectangular coordinates
            if (r < rb)
            {
                point.x = scan_cp[i]*std::cos(point_scan.angle);
                point.y = scan_cp[i]*std::sin(point_scan.angle);
                point.z = 0.00;

                // ROS_INFO("pushing point at dist %f angle %f", scan_cp[i], point_scan.angle);

                zeros_indices.push_back(i);
                bubble_point_vector.push_back(point);
            }
        }

        // These points are fine now
        bubble->addTranslation(bubble_point_vector);

        //
        // Checking for the largest non-zero sequence
        //

        // from the start of the scan to the first indexed zero of the bubble
        if (zeros_indices.front()-1 - scanStartIdx > max_sequence)
        {
            max_sequence_indices.first = scanStartIdx;
            max_sequence_indices.second = zeros_indices.front()-1;
            max_sequence = zeros_indices.front() - scanStartIdx;
        }

        // This should be thought out. If the angular
        // sweep of the bubble is greater than 50%(?) that
        // of the scan then this code is necessary
        for (size_t i = 1; i < zeros_indices.size(); i++)
        {
            if (zeros_indices[i] < scanStartIdx)
                continue;
            else if (zeros_indices[i] >= scanEndIdx)
                break;

            if (zeros_indices[i]-zeros_indices[i-1] > max_sequence)
            {
                max_sequence_indices.first = zeros_indices[i-1];
                max_sequence_indices.second = zeros_indices[i];
                max_sequence = zeros_indices[i] - zeros_indices[i-1];
            }
        }

        // Max Sequence
        // last indexed zero to the end of the scan
        if (scanEndIdx-zeros_indices.back() > max_sequence)
        {
            max_sequence_indices.first = zeros_indices.back();
            max_sequence_indices.second = scanEndIdx;
            max_sequence = scanEndIdx-scanStartIdx-zeros_indices.back();
        }

        // Find the largest point away from us within the max sequence
        auto max_point = std::make_pair(-1, msg.range_min);
        for (size_t i = max_sequence_indices.first; i < max_sequence_indices.second; i++)
        {
            if (scan_cp[i] >= max_point.second)
            {
                max_point.first = i;
                max_point.second = scan_cp[i];
            }
        }

        furthestPoint = {
            .dist = max_point.second,
            .angle = max_point.first*msg.angle_increment + msg.angle_min,
        };

        furthestPoint.p.x = furthestPoint.dist*std::cos(furthestPoint.angle);
        furthestPoint.p.y = furthestPoint.dist*std::sin(furthestPoint.angle);

        fp->addTranslation(furthestPoint.p);

        if (max_point.first < 0)
            return;

        // Set the steering angle to the farthest point
        // (TODO) Set this up to be a helper function with custom structs
        drive.header.stamp = ros::Time::now();
        drive.header.frame_id = "drive_gap_following";
        drive.drive.steering_angle = max_point.first*msg.angle_increment + msg.angle_min;
        drive.drive.steering_angle_velocity = 0.0;
        drive.drive.speed = 1.5;

        if (enabled)
            drivePub.publish(drive);
    }

    std::vector<float> find_disparities(std::vector<float> points)
    {
        float threshold = 0;

        for (size_t i = 1; i < points.size(); i++)
        {
            auto disparity = points[i] - points[i-1];
            if (disparity >= threshold)
            {
                // set virtual points
            }
        }
    }
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "gap_follow");
    GapFollowing g;

    ros::spin();
    return 0;
}

