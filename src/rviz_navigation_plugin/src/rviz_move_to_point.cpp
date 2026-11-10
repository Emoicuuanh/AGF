#include "rviz_move_to_point.h"
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <rviz/display_context.h>
#include <QHBoxLayout>

namespace rviz_move_to_point
{

MoveToPointGoal::MoveToPointGoal()
{
    _topic_property = new rviz::StringProperty( "Topic", "/move_to_point_simple/goal",
                                        "My topic on which to publish navigation goals.",
                                        getPropertyContainer(), SLOT( pub_label_point() ), this );
}

MoveToPointGoal::~MoveToPointGoal()
{
}

void MoveToPointGoal::onInitialize()
{
    rviz::PoseTool::onInitialize();
    setName("Move to point");
    pub_label_point();
}

void MoveToPointGoal::pub_label_point()
{
    _pub = _nh.advertise<geometry_msgs::PoseStamped>("/move_to_point_simple/goal", 1 );
}


void MoveToPointGoal::onPoseSet(double x, double y, double theta)
{
    //read the current fixed frame
    std::string fixed_frame = context_->getFixedFrame().toStdString();
    tf::Quaternion quat;
    quat.setRPY(0.0, 0.0, theta);

    tf::Stamped<tf::Pose> p = tf::Stamped<tf::Pose>(tf::Pose(quat, tf::Point(x, y, 0.0)), ros::Time::now(), fixed_frame);
    geometry_msgs::PoseStamped goal;
    tf::poseStampedTFToMsg(p, goal);
    
    ROS_INFO("Setting goal: Frame:%s, Position(%.3f, %.3f, %.3f), Orientation(%.3f, %.3f, %.3f, %.3f) = Angle: %.3f\n", fixed_frame.c_str(),
        goal.pose.position.x, goal.pose.position.y, goal.pose.position.z,
        goal.pose.orientation.x, goal.pose.orientation.y, goal.pose.orientation.z, goal.pose.orientation.w, theta);
    _pub.publish(goal);
}
//Override
void MoveToPointGoal::save(rviz::Config config) const
{
    rviz::PoseTool::save(config);
}

//Override
void MoveToPointGoal::load(const rviz::Config& config)
{
    rviz::PoseTool::load(config);
}

}
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_move_to_point::MoveToPointGoal, rviz::Tool)