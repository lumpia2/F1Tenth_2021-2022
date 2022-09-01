/**
 * @file wall_follow.cpp
 * @author Nathaniel Mallick (nmm109@pitt.edu)
 * @brief This file follows lab 3 of the F1Tenth lab modules (https://f1tenth.org/learn.html)
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
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Float64.h>
#include <rosbag/bag.h>

#include <f1tenth_modules/f1tenthUtils.hh>
#include <f1tenth_modules/RvizWrapper.hh>

/**
 * (todo)
 * - log P controller values
 * - need to test and tune PID controller
 */

class WallFollowing
{
    private:
        ros::NodeHandle n;
        ros::Publisher drivePub, markerPub;
        ros::Subscriber scanSub, muxSub;

        tf2_ros::Buffer tBuffer;
        tf2_ros::TransformListener tfListener;
        visualization_msgs::Marker point;
        ackermann_msgs::AckermannDriveStamped drive;
        geometry_msgs::TransformStamped baseLinkTf;

        std::string driveTopic;
        pidGains gains;
        lidarIntrinsics lidarData;
        std::unique_ptr<RvizPoint> rvizPoint;

        int muxIdx;
        int aIdx, bIdx;

        double sp, prevErr, err;
        double p,i,d;
        double L;
        double dt = 1/60.0;
        double theta = 20.0*M_PI/180.0; // [theta = 20 deg] (0 < theta < 70deg)

        bool enabled, done;

        rosbag::Bag bag;

    public:
        WallFollowing() = delete;
        WallFollowing(double rate):
            dt(1/rate),
            enabled(false), done(false),
            prevErr(0.0),
            p(0.0), i(0.0), d(0.0),
            tfListener(tBuffer),
            n(ros::NodeHandle("~"))
        {
            // Extract  lidar info from one message
            lidarData = getLidarInfoFromTopic(n, "/scan");
            if (!lidarData.valid)
                exit(-1);

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
            markerPub = n.advertise<visualization_msgs::Marker>("/dynamic_viz", 10);

            // subs
            scanSub = n.subscribe("/scan", 1, &WallFollowing::lidar_cb, this);
            muxSub = n.subscribe("/mux", 1, &WallFollowing::mux_cb, this);

            // We want this index the angle thats orthogonally
            // to the left of the front of the car _|
            bIdx = getScanIdx(M_PI/2.0, lidarData);
            aIdx = getScanIdx((M_PI/2.0)-theta, lidarData);
            ROS_INFO("Scanning data at angles %f - %f",
                lidarData.min_angle + (lidarData.scan_inc*aIdx),
                lidarData.min_angle + (lidarData.scan_inc*bIdx));

            // Update theta to be MORE accurate due to rounding errors in finding our idx
            theta = lidarData.scan_inc*(bIdx - aIdx);
            drive.drive.speed=0.0;

            geometry_msgs::Pose pose;
            geometry_msgs::Vector3 scale;

            pose.orientation.w = 1.0;
            scale.x = scale.y = 0.2;

            rvizOpts opts =
                {.color=0x00ff00, .frame_id="laser_model", .ns="point",
                 .pose=pose, .scale=scale, .topic="/dynamic_viz"};
            rvizPoint = std::make_unique<RvizPoint>(n, opts);
            rvizPoint->addTransformPair("base_link", "laser_model");
        }


        void mux_cb(const std_msgs::Int32MultiArray &msg)
        {
            // Set the mux idx to verify wether to
            //  turn the PID controller on/off.
            enabled = msg.data[muxIdx];
            done = !enabled;

            if(enabled)
            {
                ROS_INFO("PID node enabled.");
                bag.open("/home/ras/F1Tenth/F1Tenth_2021-2022/catkin_ws/src/f1tenth_modules/bags/PID_info.bag", rosbag::bagmode::Write);
            }
            else
            {
                ROS_INFO("PID node disabled.");
                bag.close();
            }

            // (TODO) Maybe reset the PID values when "else"
        }

        void lidar_cb(const sensor_msgs::LaserScan &msg)
        {
            /////!!!!! NEED TO FILTER FOR BAD DISTANCES (inf &&&& <0)
            auto a = msg.ranges[aIdx];
            auto b = msg.ranges[bIdx];

            geometry_msgs::Point point_a, point_b;

            auto a_angle = aIdx*msg.angle_increment + msg.angle_min;
            auto b_angle = bIdx*msg.angle_increment + msg.angle_min;

            point_a.x = msg.ranges[aIdx]*std::cos(a_angle);
            point_a.y = msg.ranges[aIdx]*std::sin(a_angle);

            point_b.x = msg.ranges[bIdx]*std::cos(b_angle);
            point_b.y = msg.ranges[bIdx]*std::sin(b_angle);

            point_a.z = point_b.z = 0.0;
            std::vector<geometry_msgs::Point> points = {point_a, point_b};

            rvizPoint->addTranslation(points);

            auto alpha = std::atan((a*std::cos(theta)-b)/(a*std::sin(theta)));
            auto dist_1 = (b*std::cos(alpha)) + (drive.drive.speed*dt)*std::sin(alpha);

            err = sp - dist_1;

            if(enabled)
                pid_control(err);

            prevErr = err;
        }

        void pid_control(const double &err)
        {
            ROS_INFO("Error: %f", err);
            p = err;
            i += err*dt; // may need to be clamped
            d = (err-prevErr)/dt;

            std_msgs::Float64 p_m;
            std_msgs::Float64 i_m;
            std_msgs::Float64 d_m;
            std_msgs::Float64 err_m;

            p_m.data = gains.kp;
            i_m.data = gains.ki;
            d_m.data = gains.kd;
            err_m.data = err;

            bag.write("P", ros::Time::now(), p_m);
            bag.write("I", ros::Time::now(), i_m);
            bag.write("D", ros::Time::now(), d_m);
            bag.write("Error", ros::Time::now(), err_m);

            const auto steer_angle = -(gains.kp*p + gains.ki*i + gains.kd*d);
            const auto steer_ang_deg = steer_angle*(180.0/M_PI);
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
    WallFollowing w(60.0);
    ros::Rate rate(w.getRate());

    while(!w.isDone())
    {
        w.spin();
        rate.sleep();
    }
}
