#ifndef RVIZ_WRAPPER_HH
#define RVIZ_WRAPPER_HH

#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <f1tenth_modules/f1tenthUtils.hh>

/*
    TODO (NMM) Need API documentation
*/
class RvizWrapper
{
protected:
    ros::Publisher markerPub;
    visualization_msgs::Marker marker;

    std::unique_ptr<tf2_ros::Buffer> tfBuffer;
    std::unique_ptr<tf2_ros::TransformListener> tfListener;
    std::pair<std::string, std::string> transformPair;

public:
    static int32_t id;
    RvizWrapper();
    virtual void changeColor(uint32_t);
    virtual void changeScale(const geometry_msgs::Vector3 &);
    virtual void addTransformPair(const std::string &, const std::string &);
    virtual void addTranslation(const geometry_msgs::Point &);
    virtual void addTranslation(const std::vector<geometry_msgs::Point> &);
};

class RvizPoint : public RvizWrapper
{
private:

public:
    RvizPoint() = delete;
    RvizPoint(ros::NodeHandle &, const rvizOpts &);
};


#endif