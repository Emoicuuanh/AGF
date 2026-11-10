#include <odometry_calculate/odometry_forklift.h>

namespace odometry_forklift
{
    OdometryForklift::OdometryForklift(ros::NodeHandle nh): nh_(nh)
    {
        steering_angle_sub_ = nh_.subscribe("/steering_angle", 1, &OdometryForklift::steering_angle_callback,this);
        sub_odom_ =  nh_.subscribe("/odom", 20, &OdometryForklift::odom_callback, this);

        x_odom_ = 0.0;
        y_odom_ = 0.0;
        theta_robot_ = 0.0;
    }

    OdometryForklift::~OdometryForklift() {}

    void OdometryForklift::steering_angle_callback(const std_msgs::Float64::ConstPtr &msg)
    {
        steering_angle_ = *msg;
    }

    void OdometryForklift::odom_callback(const nav_msgs::Odometry::ConstPtr &msg)
    {
        double linear = msg->twist.twist.linear.x;
        double angular_z = msg->twist.twist.angular.z;

        double current_time = ros::Time::now().toSec();
        double dt = current_time - last_time_;
        last_time_ = current_time;
        if (steering_angle_.data != 100000.0)
        {
            double delta_linear = linear * dt;
            double delta_z = angular_z * dt;
            theta_robot_ += delta_z;
            theta_robot_ = normalizeAngle(theta_robot_);
            x_odom_ += delta_linear * cos(theta_robot_);
            y_odom_ += delta_linear * sin(theta_robot_);
        }
    }

    double OdometryForklift::normalizeAngle(double angle)
    {
        while (angle > M_PI)
        {
            angle -= 2 * M_PI;
        }
        while (angle < -M_PI)
        {
            angle += 2 * M_PI;
        }
        return angle;
    }

    void OdometryForklift::calculate_odom_laser()
    {
        while(ros::ok())
        {
            ros::spinOnce();
            std::cout << "x_odom: " << x_odom_ << std::endl;
            std::cout << "y_odom: " << y_odom_ << std::endl;
            std::cout << "theta_robot: " << theta_robot_ << std::endl;
            ros::Duration(0.1).sleep();
        }
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "odometry_forklift");
    ros::NodeHandle nh;
    odometry_forklift::OdometryForklift odometry_forklift(nh);
    odometry_forklift.calculate_odom_laser();
    ros::spin();
    return 0;
}
