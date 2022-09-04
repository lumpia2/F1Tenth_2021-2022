#include <ros/ros.h>
#include <std_msgs/String.h>

#include <termios.h>
#include <stdio.h>
#include <signal.h>

static volatile sig_atomic_t done = 0;

void handlr(int sig)
{
    done = 1;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Input");
    ros::NodeHandle n = ros::NodeHandle("~");

    // TODO Maybe we can configure a specific input
    // topic but for now manually specific it.
    std::string inputTopic{"/input"};

    ros::Publisher inputPub = n.advertise<std_msgs::String>(inputTopic, 10);

    // Terminal Interface
    static struct termios curr_t, new_t;
    tcgetattr(STDIN_FILENO, &curr_t);
    new_t = curr_t;
    new_t.c_lflag &= ~ICANON;
    tcsetattr(STDIN_FILENO, 0, &new_t);

    struct sigaction act;
    act.sa_handler = handlr;
    sigaction(SIGINT, &act, NULL);

    char c;
    std_msgs::String msg;

    while (ros::ok() && !done)
    {
        c = getchar();

        msg.data = c;
        inputPub.publish(msg);
    }

    // Reset the terminal mode to what it was before
    tcsetattr(STDIN_FILENO, 0, &curr_t);

    return 0;
}