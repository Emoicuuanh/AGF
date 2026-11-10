#include "rviz_estimate_label.h"
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <rviz/display_context.h>
#include <QHBoxLayout>

namespace rviz_estimate_label
{

LabelEstimateTool::LabelEstimateTool()
{
    _topic_property = new rviz::StringProperty( "Topic", "/my_pose_estimate",
                                        "My topic on which to publish navigation goals.",
                                        getPropertyContainer(), SLOT( pub_label_point() ), this );
}

LabelEstimateTool::~LabelEstimateTool()
{
}

void LabelEstimateTool::onInitialize()
{
    rviz::PoseTool::onInitialize();
    setName("Label Pose Estimate");
    pub_label_point();
}

void LabelEstimateTool::pub_label_point()
{
    _pub = _nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/my_pose_estimate", 1 );
}


void LabelEstimateTool::onPoseSet(double x, double y, double theta)
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
void LabelEstimateTool::save(rviz::Config config) const
{
    rviz::PoseTool::save(config);
}

//Override
void LabelEstimateTool::load(const rviz::Config& config)
{
    rviz::PoseTool::load(config);
}

}
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_estimate_label::LabelEstimateTool, rviz::Tool)