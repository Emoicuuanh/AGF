#ifndef RIVZ_POINT_LABEL
#define RIVZ_POINT_LABEL

#ifndef Q_MOC_RUN

#include <ros/ros.h>
#include <rviz/properties/string_property.h>
#include <rviz/default_plugin/tools/pose_tool.h>
#include <QLineEdit>

#endif

namespace rviz_rotate_diff
{

class RotateDiffGoal: public rviz::PoseTool
{
Q_OBJECT
public:
    RotateDiffGoal();
    ~RotateDiffGoal();

    virtual void onInitialize();
    virtual void load( const rviz::Config& config );
    virtual void save( rviz::Config config ) const;

private Q_SLOTS:
    void pub_label_point();

protected:
    virtual void onPoseSet(double x,double y, double theta);            

private:
    // publish the labelled point
    ros::NodeHandle _nh;
    ros::Publisher _pub;
    rviz::StringProperty *_topic_property;
};

}

#endif