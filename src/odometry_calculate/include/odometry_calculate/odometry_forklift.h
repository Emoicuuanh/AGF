#ifndef ODOMETRY_FORLIFT_H_
#define ODOMETRY_FORLIFT_H_

#include <ros/ros.h>

#include <Eigen/Geometry>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <agf_msgs/Int8Array.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Int8.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <std_srvs/SetBool.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_stamped_msgs/StringStamped.h>

namespace odometry_forklift
{
    class OdometryForklift
    {
        public:
            OdometryForklift(ros::NodeHandle nh);

            ~OdometryForklift();

            void odom_callback(const nav_msgs::Odometry::ConstPtr &msg);

            void steering_angle_callback(const std_msgs::Float64::ConstPtr &msg);

            double normalizeAngle(double angle);

            void calculate_odom_laser();

        private:
            ros::NodeHandle nh_;

            ros::Subscriber steering_angle_sub_;
            ros::Subscriber sub_odom_;

            std_msgs::Float64 steering_angle_;

            double theta_robot_;
            double x_odom_;
            double y_odom_;
            double last_time_;
    };
};
#endif