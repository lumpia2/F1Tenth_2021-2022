// MAIN FUNCTION FOR MUX NODE
#include <ros/ros.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <std_msgs/UInt8.h>
#include <f1tenth_modules/States.hh>

class Mux
{
private:
    // ROS
    ros::NodeHandle n;
    ros::Publisher muxOut;
    ros::Subscriber muxIn;
    ros::Subscriber muxController;

    std::string currState;

    // main drive topic publisher
    void muxIn_cb(const ackermann_msgs::AckermannDriveStamped &msg)
    {
        muxOut.publish(msg);
    }

    void brake()
    {
        ackermann_msgs::AckermannDriveStamped drive;
        drive.header.stamp = ros::Time::now();
        drive.header.frame_id = States::Off::NAME;

        drive.drive.speed = 0.0;

        muxOut.publish(drive);
    }

    void switch_cb(const std_msgs::UInt8 &msg)
    {
        // For now, need to manually add a case statement
        // for every new state added to the system.
        switch(msg.data)
        {
            case States::Off::INPUT_CHAR:
                if (currState == States::Off::NAME)
                    break;

                muxIn.shutdown();
                brake();
                ROS_INFO("Switching to state %s", States::Off::NAME);
                currState = States::Off::NAME;

                break;
            case States::Manual::INPUT_CHAR:
                if (currState == States::Manual::NAME)
                    break;

                muxIn.shutdown();
                muxIn = n.subscribe(States::Manual::DRIVE_TOPIC, 1, &Mux::muxIn_cb, this);
                ROS_INFO("Switching to state %s", States::Manual::NAME);
                currState = States::Manual::NAME;

                break;
            case States::WallFollowing::INPUT_CHAR:
                if (currState == States::WallFollowing::NAME)
                    break;

                muxIn.shutdown();
                muxIn = n.subscribe(States::WallFollowing::DRIVE_TOPIC, 1, &Mux::muxIn_cb, this);
                ROS_INFO("Switching to state %s", States::WallFollowing::NAME);
                currState = States::WallFollowing::NAME;

                break;
            case States::GapFollowing::INPUT_CHAR:
                if (currState == States::GapFollowing::NAME)
                    break;

                muxIn.shutdown();
                muxIn = n.subscribe(States::GapFollowing::DRIVE_TOPIC, 1, &Mux::muxIn_cb, this);
                ROS_INFO("Switching to state %s", States::GapFollowing::NAME);
                currState = States::GapFollowing::NAME;

                break;
            default:
                muxIn.shutdown();
                brake();
                ROS_ERROR("Unkown state change : (%d)", msg.data);
                currState == "ERR";

                break;
        }
    }

public:
    Mux()
    {
        // ROS NODE
        n = ros::NodeHandle("~");

        // Subcribers
        muxController = n.subscribe("/input", 1, &Mux::switch_cb, this);
        muxIn = n.subscribe("", 1, &Mux::switch_cb, this);

        // Publishers
        muxOut = n.advertise<ackermann_msgs::AckermannDriveStamped>("/drive", 1);
    }
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "mux_controller");
    Mux m;
    ros::spin();
    return 0;
}