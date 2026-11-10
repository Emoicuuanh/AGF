#include "rviz_set_waypoint.h"
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <rviz/display_context.h>
#include <QHBoxLayout>

namespace rviz_set_waypoint
{

SetWaypointTool::SetWaypointTool()
{
    _topic_property = new rviz::StringProperty( "Topic", "/set_waypoint",
                                        "My topic on which to publish navigation goals.",
                                        getPropertyContainer(), SLOT( pub_label_point() ), this );
}

SetWaypointTool::~SetWaypointTool()
{
}

void SetWaypointTool::onInitialize()
{
    rviz::PoseTool::onInitialize();
    setName("Set Waypoint");
    pub_label_point();
}

void SetWaypointTool::pub_label_point()
{
    _pub = _nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/set_waypoint", 1 );
}


void SetWaypointTool::onPoseSet(double x, double y, double theta)
{
    //read the current fixed frame
    std::string fixed_frame = context_->getFixedFrame().toStdString();

    geometry_msgs::PoseWithCovarianceStamped goal;
    goal.header.frame_id = fixed_frame;
    goal.header.stamp = ros::Time::now();

    // set x,y coord
    goal.pose.pose.position.x = x;
    goal.pose.pose.position.y = y;
    goal.pose.pose.position.z = 0.0;

    // set theta
    tf::Quaternion quat;
    quat.setRPY(0.0, 0.0, theta);
    tf::quaternionTFToMsg(quat, goal.pose.pose.orientation);
    // goal.pose.covariance[6*0+0] = 0.5 * 0.5;
    // goal.pose.covariance[6*1+1] = 0.5 * 0.5;
    // goal.pose.covariance[6*5+5] = M_PI/12.0 * M_PI/12.0;
    
    ROS_INFO("Setting estimate: Frame:%s, Position(%.3f, %.3f, %.3f), Orientation(%.3f, %.3f, %.3f, %.3f) = Angle: %.3f\n", fixed_frame.c_str(),
        goal.pose.pose.position.x, goal.pose.pose.position.y, goal.pose.pose.position.z,
        goal.pose.pose.orientation.x, goal.pose.pose.orientation.y, goal.pose.pose.orientation.z, goal.pose.pose.orientation.w, theta);
    _pub.publish(goal);
}
//Override
void SetWaypointTool::save(rviz::Config config) const
{
    rviz::PoseTool::save(config);
}

//Override
void SetWaypointTool::load(const rviz::Config& config)
{
    rviz::PoseTool::load(config);
}

}
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_set_waypoint::SetWaypointTool, rviz::Tool)