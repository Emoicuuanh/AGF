/*!
 * \file robot_pose_publisher.cpp
 * \brief Publishes the robot's position in a geometry_msgs/Pose message.
 *
 * Publishes the robot's position in a geometry_msgs/Pose message based on the TF
 * difference between /map and /base_link.
 *
 * \author Russell Toris - rctoris@wpi.edu
 * \date April 3, 2014
 */
#include <ros/ros.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

/*!
 * Creates and runs the robot_pose_publisher node.
 *
 * \param argc argument count that is passed to ros::init
 * \param argv arguments that are passed to ros::init
 * \return EXIT_SUCCESS if the node runs correctly
 */
int main(int argc, char** argv)
{
  // initialize ROS and the node
  ros::init(argc, argv, "robot_pose_publisher");
  ros::NodeHandle nh;
  ros::NodeHandle nh_priv("~");

  // configuring parameters
  std::string map_frame, base_frame, laser_frame, laser_map_frame;
  double publish_frequency;
  bool is_stamped;
  ros::Publisher p_pub, p_pub1, p_pub2;

  nh_priv.param<std::string>("map_frame", map_frame, "map");
  nh_priv.param<std::string>("base_frame", base_frame, "base_link");
  nh_priv.param<std::string>("laser_frame", laser_frame, "laser");
  nh_priv.param<std::string>("laser_map_frame", laser_map_frame, "laser");
  nh_priv.param<double>("publish_frequency", publish_frequency, 10);
  nh_priv.param<bool>("is_stamped", is_stamped, false);

  if (is_stamped)
  {
    p_pub = nh.advertise<geometry_msgs::PoseStamped>("robot_pose", 1);
    p_pub1 = nh.advertise<geometry_msgs::PoseStamped>("laser_pose", 1);
    p_pub2 = nh.advertise<geometry_msgs::PoseStamped>("laser_map_pose", 1);
  }
  else
  {
    p_pub = nh.advertise<geometry_msgs::Pose>("robot_pose", 1);
    p_pub1 = nh.advertise<geometry_msgs::Pose>("laser_pose", 1);
    p_pub2 = nh.advertise<geometry_msgs::Pose>("laser_map_pose", 1);
  }

  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener(tf_buffer);
  geometry_msgs::TransformStamped transfrom_robot, transfrom_laser, transfrom_laser_map;

  ros::Rate rate(publish_frequency);
  while (nh.ok())
  {
    try
    {
      // FIXME: Bug cannot lockup /laser_map_frame if /laser_frame fail
      transfrom_robot = tf_buffer.lookupTransform(map_frame, base_frame, ros::Time(0),
                               ros::Duration(3.0));
      transfrom_laser_map = tf_buffer.lookupTransform(map_frame, laser_map_frame, ros::Time(0),
                               ros::Duration(3.0));
      transfrom_laser = tf_buffer.lookupTransform(map_frame, laser_frame, ros::Time(0),
                               ros::Duration(3.0));

      // Construct a pose message
      geometry_msgs::PoseStamped pose_stamped, pose_stamped1, pose_stamped2;
      // Robot pose msg
      pose_stamped.header.frame_id = map_frame;
      pose_stamped.header.stamp = ros::Time::now();

      pose_stamped.pose.orientation.x = transfrom_robot.transform.rotation.x;
      pose_stamped.pose.orientation.y = transfrom_robot.transform.rotation.y;
      pose_stamped.pose.orientation.z = transfrom_robot.transform.rotation.z;
      pose_stamped.pose.orientation.w = transfrom_robot.transform.rotation.w;

      pose_stamped.pose.position.x = transfrom_robot.transform.translation.x;
      pose_stamped.pose.position.y = transfrom_robot.transform.translation.y;
      pose_stamped.pose.position.z = transfrom_robot.transform.translation.z;

      // Lase pose msg
      pose_stamped1.header.frame_id = map_frame;
      pose_stamped1.header.stamp = ros::Time::now();

      pose_stamped1.pose.orientation.x = transfrom_laser.transform.rotation.x;
      pose_stamped1.pose.orientation.y = transfrom_laser.transform.rotation.y;
      pose_stamped1.pose.orientation.z = transfrom_laser.transform.rotation.z;
      pose_stamped1.pose.orientation.w = transfrom_laser.transform.rotation.w;

      pose_stamped1.pose.position.x = transfrom_laser.transform.translation.x;
      pose_stamped1.pose.position.y = transfrom_laser.transform.translation.y;
      pose_stamped1.pose.position.z = transfrom_laser.transform.translation.z;

      // Robot pose msg
      pose_stamped2.header.frame_id = map_frame;
      pose_stamped2.header.stamp = ros::Time::now();

      pose_stamped2.pose.orientation.x =
          transfrom_laser_map.transform.rotation.x;
      pose_stamped2.pose.orientation.y =
          transfrom_laser_map.transform.rotation.y;
      pose_stamped2.pose.orientation.z =
          transfrom_laser_map.transform.rotation.z;
      pose_stamped2.pose.orientation.w =
          transfrom_laser_map.transform.rotation.w;

      pose_stamped2.pose.position.x = transfrom_laser_map.transform.translation.x;
      pose_stamped2.pose.position.y = transfrom_laser_map.transform.translation.y;
      pose_stamped2.pose.position.z = transfrom_laser_map.transform.translation.z;

      if (is_stamped)
      {
        p_pub.publish(pose_stamped);
        p_pub1.publish(pose_stamped1);
        p_pub2.publish(pose_stamped2);
      }
      else
      {
        p_pub.publish(pose_stamped.pose);
        p_pub1.publish(pose_stamped1.pose);
        p_pub2.publish(pose_stamped2.pose);
      }
    }
    catch (tf2::TransformException& ex)
    {
      // just continue on
    }

    rate.sleep();
  }

  return EXIT_SUCCESS;
}
