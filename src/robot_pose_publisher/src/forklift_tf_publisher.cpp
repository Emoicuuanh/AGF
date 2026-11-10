#include <ros/ros.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>

nav_msgs::Odometry odom_nav;
bool nav_status = false;
// mirror_ok = true  → gương OK, NAV350 publish map→odom 0,0,0 / odom->nav_footprint 
// mirror_ok = false → mất gương, MRPT ICP take over map→odom
bool mirror_ok = true;

void laserCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  odom_nav = *msg;
  nav_status = true;
}
void mirrorCallback(const std_msgs::Bool::ConstPtr& msg)
{
  // mirror_detect = true có nghĩa là MẤT gương
  mirror_ok = !msg->data;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "forklift_tf_publisher");
  ros::NodeHandle nh;
  ros::NodeHandle nh_priv("~");

  double map_odom_angle, laser_base_x, laser_base_y, laser_base_z, laser_base_angle_yaw,
      wheel_radius, turning_radius, steer_wheel_x, steer_wheel_y, laser_1_x, laser_1_y, laser_1_angle_yaw,
      laser_2_x, laser_2_y, laser_2_angle_yaw;
  nh_priv.param<double>("map_odom_angle", map_odom_angle, 0.0);
  nh_priv.param<double>("laser_base_x", laser_base_x, 0.0);
  nh_priv.param<double>("laser_base_y", laser_base_y, 0.0);
  nh_priv.param<double>("laser_base_z", laser_base_z, 0.0);
  nh_priv.param<double>("laser_base_angle_yaw", laser_base_angle_yaw, 0.0);
  nh_priv.param<double>("wheel_radius", wheel_radius, 0.0);
  nh_priv.param<double>("turning_radius", turning_radius, 0.0);
  nh_priv.param<double>("steer_wheel_x", steer_wheel_x, 0.0);
  nh_priv.param<double>("steer_wheel_y", steer_wheel_y, 0.0);
  nh_priv.param<double>("laser_1_x", laser_1_x, 0.0);
  nh_priv.param<double>("laser_1_y", laser_1_y, 0.0);
  nh_priv.param<double>("laser_1_angle_yaw", laser_1_angle_yaw, 0.0);
  nh_priv.param<double>("laser_2_x", laser_2_x, 0.0);
  nh_priv.param<double>("laser_2_y", laser_2_y, 0.0);
  nh_priv.param<double>("laser_2_angle_yaw", laser_2_angle_yaw, 0.0);

  ros::Subscriber sub_nav = nh.subscribe("/nav350laser/odom", 1, laserCallback);
  // Nhận tín hiệu mất gương từ NAV350
  ros::Subscriber sub_mirror = nh.subscribe("/mirror_detect", 1, mirrorCallback);

  static tf2_ros::TransformBroadcaster br1;
  static tf2_ros::TransformBroadcaster br2;
  static tf2_ros::TransformBroadcaster br3;
  static tf2_ros::TransformBroadcaster br4;
  static tf2_ros::TransformBroadcaster br5;
  static tf2_ros::TransformBroadcaster br6;
  static tf2_ros::TransformBroadcaster br7;
  static tf2_ros::TransformBroadcaster br8;
  static tf2_ros::TransformBroadcaster br9;
  static tf2_ros::TransformBroadcaster br10;

  tf2::Quaternion Quaternion1;
  tf2::Quaternion Quaternion2;
  tf2::Quaternion Quaternion3;
  tf2::Quaternion Quaternion4;
  tf2::Quaternion Quaternion5;
  tf2::Quaternion Quaternion6;
  tf2::Quaternion Quaternion7;
  tf2::Quaternion Quaternion8;
  tf2::Quaternion Quaternion9;
  tf2::Quaternion Quaternion10;

  geometry_msgs::TransformStamped transformStamped1;
  geometry_msgs::TransformStamped transformStamped2;
  geometry_msgs::TransformStamped transformStamped3;
  geometry_msgs::TransformStamped transformStamped4;
  geometry_msgs::TransformStamped transformStamped5;
  geometry_msgs::TransformStamped transformStamped6;
  geometry_msgs::TransformStamped transformStamped7;
  geometry_msgs::TransformStamped transformStamped8;
  geometry_msgs::TransformStamped transformStamped9;
  geometry_msgs::TransformStamped transformStamped10;

  Quaternion1.setRPY(0, 0, map_odom_angle);
  transformStamped1.header.frame_id = "map";
  transformStamped1.child_frame_id = "odom";

  transformStamped1.transform.translation.x = 0;
  transformStamped1.transform.translation.y = 0;
  transformStamped1.transform.translation.z = 0;

  transformStamped1.transform.rotation.x = Quaternion1.getX();
  transformStamped1.transform.rotation.y = Quaternion1.getY();
  transformStamped1.transform.rotation.z = Quaternion1.getZ();
  transformStamped1.transform.rotation.w = Quaternion1.getW();

  Quaternion2.setRPY(0, 0, laser_base_angle_yaw);
  transformStamped2.header.frame_id = "nav_footprint";
  transformStamped2.child_frame_id = "base_footprint";

  transformStamped2.transform.translation.x = laser_base_x;
  transformStamped2.transform.translation.y = laser_base_y;
  transformStamped2.transform.translation.z = 0;

  transformStamped2.transform.rotation.x = Quaternion2.getX();
  transformStamped2.transform.rotation.y = Quaternion2.getY();
  transformStamped2.transform.rotation.z = Quaternion2.getZ();
  transformStamped2.transform.rotation.w = Quaternion2.getW();

  Quaternion3.setRPY(0, 0, 0);
  transformStamped3.header.frame_id = "base_footprint";
  transformStamped3.child_frame_id = "base_link";

  transformStamped3.transform.translation.x = 0;
  transformStamped3.transform.translation.y = 0;
  transformStamped3.transform.translation.z = wheel_radius;

  transformStamped3.transform.rotation.x = Quaternion3.getX();
  transformStamped3.transform.rotation.y = Quaternion3.getY();
  transformStamped3.transform.rotation.z = Quaternion3.getZ();
  transformStamped3.transform.rotation.w = Quaternion3.getW();

  Quaternion4.setRPY(0, 0, 0);
  transformStamped4.header.frame_id = "base_link";
  transformStamped4.child_frame_id = "right_wheel";

  transformStamped4.transform.translation.x = 0;
  transformStamped4.transform.translation.y = -turning_radius;
  transformStamped4.transform.translation.z = 0;

  transformStamped4.transform.rotation.x = Quaternion4.getX();
  transformStamped4.transform.rotation.y = Quaternion4.getY();
  transformStamped4.transform.rotation.z = Quaternion4.getZ();
  transformStamped4.transform.rotation.w = Quaternion4.getW();

  Quaternion5.setRPY(0, 0, 0);
  transformStamped5.header.frame_id = "base_link";
  transformStamped5.child_frame_id = "left_wheel";

  transformStamped5.transform.translation.x = 0;
  transformStamped5.transform.translation.y = turning_radius;
  transformStamped5.transform.translation.z = 0;

  transformStamped5.transform.rotation.x = Quaternion5.getX();
  transformStamped5.transform.rotation.y = Quaternion5.getY();
  transformStamped5.transform.rotation.z = Quaternion5.getZ();
  transformStamped5.transform.rotation.w = Quaternion5.getW();

  Quaternion6.setRPY(0, 0, 0);
  transformStamped6.header.frame_id = "base_link";
  transformStamped6.child_frame_id = "steer_wheel";

  transformStamped6.transform.translation.x = steer_wheel_x;
  transformStamped6.transform.translation.y = steer_wheel_y;
  transformStamped6.transform.translation.z = 0;

  transformStamped6.transform.rotation.x = Quaternion6.getX();
  transformStamped6.transform.rotation.y = Quaternion6.getY();
  transformStamped6.transform.rotation.z = Quaternion6.getZ();
  transformStamped6.transform.rotation.w = Quaternion6.getW();

  Quaternion7.setRPY(0, 0, laser_base_angle_yaw);
  transformStamped7.header.stamp = ros::Time::now();
  transformStamped7.header.frame_id = "nav_footprint";
  transformStamped7.child_frame_id = "laser_map";

  transformStamped7.transform.translation.x = 0;
  transformStamped7.transform.translation.y = 0;
  transformStamped7.transform.translation.z = laser_base_z;

  transformStamped7.transform.rotation.x = Quaternion7.getX();
  transformStamped7.transform.rotation.y = Quaternion7.getY();
  transformStamped7.transform.rotation.z = Quaternion7.getZ();
  transformStamped7.transform.rotation.w = Quaternion7.getW();

  Quaternion9.setRPY(0, 0, laser_1_angle_yaw);
  transformStamped9.header.frame_id = "base_link";
  transformStamped9.child_frame_id = "laser_1";

  transformStamped9.transform.translation.x = laser_1_x;
  transformStamped9.transform.translation.y = laser_1_y;
  transformStamped9.transform.translation.z = 0;

  transformStamped9.transform.rotation.x = Quaternion9.getX();
  transformStamped9.transform.rotation.y = Quaternion9.getY();
  transformStamped9.transform.rotation.z = Quaternion9.getZ();
  transformStamped9.transform.rotation.w = Quaternion9.getW();

  Quaternion10.setRPY(0, 0, laser_2_angle_yaw);
  transformStamped10.header.frame_id = "base_link";
  transformStamped10.child_frame_id = "laser_2";

  transformStamped10.transform.translation.x = laser_2_x;
  transformStamped10.transform.translation.y = laser_2_y;
  transformStamped10.transform.translation.z = 0;

  transformStamped10.transform.rotation.x = Quaternion10.getX();
  transformStamped10.transform.rotation.y = Quaternion10.getY();
  transformStamped10.transform.rotation.z = Quaternion10.getZ();
  transformStamped10.transform.rotation.w = Quaternion10.getW();

  while (ros::ok())
  {
    ros::spinOnce();
    if (nav_status)
    {
      if (mirror_ok)
      {
        // ── GƯƠNG OK ──────────────────────────────────────────────────────
        br1.sendTransform(transformStamped1);
      }
      transformStamped8.header.stamp = ros::Time::now();
      transformStamped8.header.frame_id = "odom";
      transformStamped8.child_frame_id = "nav_footprint";

      transformStamped8.transform.translation.x = odom_nav.pose.pose.position.x;
      transformStamped8.transform.translation.y = odom_nav.pose.pose.position.y;
      transformStamped8.transform.translation.z = odom_nav.pose.pose.position.z;

      transformStamped8.transform.rotation.x = odom_nav.pose.pose.orientation.x;
      transformStamped8.transform.rotation.y = odom_nav.pose.pose.orientation.y;
      transformStamped8.transform.rotation.z = odom_nav.pose.pose.orientation.z;
      transformStamped8.transform.rotation.w = odom_nav.pose.pose.orientation.w;
      br8.sendTransform(transformStamped8);
    }
    transformStamped1.header.stamp = ros::Time::now();
    transformStamped2.header.stamp = ros::Time::now();
    transformStamped3.header.stamp = ros::Time::now();
    transformStamped4.header.stamp = ros::Time::now();
    transformStamped5.header.stamp = ros::Time::now();
    transformStamped6.header.stamp = ros::Time::now();
    transformStamped7.header.stamp = ros::Time::now();
    transformStamped9.header.stamp = ros::Time::now();
    transformStamped10.header.stamp = ros::Time::now();



    // br1.sendTransform(transformStamped1);
    br2.sendTransform(transformStamped2);
    br3.sendTransform(transformStamped3);
    br4.sendTransform(transformStamped4);
    br5.sendTransform(transformStamped5);
    br6.sendTransform(transformStamped6);
    br7.sendTransform(transformStamped7);
    // br9.sendTransform(transformStamped9);
    // br10.sendTransform(transformStamped10);
    ros::Duration(0.05).sleep();
  }
}













// #include <ros/ros.h>

// #include <tf2_geometry_msgs/tf2_geometry_msgs.h>
// #include <tf2_ros/transform_broadcaster.h>
// #include <tf2_ros/transform_listener.h>

// #include <geometry_msgs/TransformStamped.h>
// #include <nav_msgs/Odometry.h>

// nav_msgs::Odometry odom_nav;
// bool nav_status = false;

// void laserCallback(const nav_msgs::Odometry::ConstPtr& msg)
// {
//   odom_nav = *msg;
//   nav_status = true;
// }

// int main(int argc, char** argv)
// {
//   ros::init(argc, argv, "forklift_tf_publisher");
//   ros::NodeHandle nh;
//   ros::NodeHandle nh_priv("~");

//   double map_odom_angle, laser_base_x, laser_base_y, laser_base_z, laser_base_angle_yaw,
//       wheel_radius, turning_radius, steer_wheel_x, steer_wheel_y, laser_1_x, laser_1_y, laser_1_angle_yaw,
//       laser_2_x, laser_2_y, laser_2_angle_yaw;
//   nh_priv.param<double>("map_odom_angle", map_odom_angle, 0.0);
//   nh_priv.param<double>("laser_base_x", laser_base_x, 0.0);
//   nh_priv.param<double>("laser_base_y", laser_base_y, 0.0);
//   nh_priv.param<double>("laser_base_z", laser_base_z, 0.0);
//   nh_priv.param<double>("laser_base_angle_yaw", laser_base_angle_yaw, 0.0);
//   nh_priv.param<double>("wheel_radius", wheel_radius, 0.0);
//   nh_priv.param<double>("turning_radius", turning_radius, 0.0);
//   nh_priv.param<double>("steer_wheel_x", steer_wheel_x, 0.0);
//   nh_priv.param<double>("steer_wheel_y", steer_wheel_y, 0.0);
//   nh_priv.param<double>("laser_1_x", laser_1_x, 0.0);
//   nh_priv.param<double>("laser_1_y", laser_1_y, 0.0);
//   nh_priv.param<double>("laser_1_angle_yaw", laser_1_angle_yaw, 0.0);
//   nh_priv.param<double>("laser_2_x", laser_2_x, 0.0);
//   nh_priv.param<double>("laser_2_y", laser_2_y, 0.0);
//   nh_priv.param<double>("laser_2_angle_yaw", laser_2_angle_yaw, 0.0);

//   ros::Subscriber sub_nav = nh.subscribe("/nav350laser/odom", 1, laserCallback);

//   static tf2_ros::TransformBroadcaster br1;
//   static tf2_ros::TransformBroadcaster br2;
//   static tf2_ros::TransformBroadcaster br3;
//   static tf2_ros::TransformBroadcaster br4;
//   static tf2_ros::TransformBroadcaster br5;
//   static tf2_ros::TransformBroadcaster br6;
//   static tf2_ros::TransformBroadcaster br7;
//   static tf2_ros::TransformBroadcaster br8;
//   static tf2_ros::TransformBroadcaster br9;
//   static tf2_ros::TransformBroadcaster br10;

//   tf2::Quaternion Quaternion1;
//   tf2::Quaternion Quaternion2;
//   tf2::Quaternion Quaternion3;
//   tf2::Quaternion Quaternion4;
//   tf2::Quaternion Quaternion5;
//   tf2::Quaternion Quaternion6;
//   tf2::Quaternion Quaternion7;
//   tf2::Quaternion Quaternion8;
//   tf2::Quaternion Quaternion9;
//   tf2::Quaternion Quaternion10;

//   geometry_msgs::TransformStamped transformStamped1;
//   geometry_msgs::TransformStamped transformStamped2;
//   geometry_msgs::TransformStamped transformStamped3;
//   geometry_msgs::TransformStamped transformStamped4;
//   geometry_msgs::TransformStamped transformStamped5;
//   geometry_msgs::TransformStamped transformStamped6;
//   geometry_msgs::TransformStamped transformStamped7;
//   geometry_msgs::TransformStamped transformStamped8;
//   geometry_msgs::TransformStamped transformStamped9;
//   geometry_msgs::TransformStamped transformStamped10;

//   Quaternion1.setRPY(0, 0, map_odom_angle);
//   transformStamped1.header.frame_id = "map";
//   transformStamped1.child_frame_id = "odom";

//   transformStamped1.transform.translation.x = 0;
//   transformStamped1.transform.translation.y = 0;
//   transformStamped1.transform.translation.z = 0;

//   transformStamped1.transform.rotation.x = Quaternion1.getX();
//   transformStamped1.transform.rotation.y = Quaternion1.getY();
//   transformStamped1.transform.rotation.z = Quaternion1.getZ();
//   transformStamped1.transform.rotation.w = Quaternion1.getW();

//   Quaternion2.setRPY(0, 0, laser_base_angle_yaw);
//   transformStamped2.header.frame_id = "nav_footprint";
//   transformStamped2.child_frame_id = "base_footprint";

//   transformStamped2.transform.translation.x = laser_base_x;
//   transformStamped2.transform.translation.y = laser_base_y;
//   transformStamped2.transform.translation.z = 0;

//   transformStamped2.transform.rotation.x = Quaternion2.getX();
//   transformStamped2.transform.rotation.y = Quaternion2.getY();
//   transformStamped2.transform.rotation.z = Quaternion2.getZ();
//   transformStamped2.transform.rotation.w = Quaternion2.getW();

//   Quaternion3.setRPY(0, 0, 0);
//   transformStamped3.header.frame_id = "base_footprint";
//   transformStamped3.child_frame_id = "base_link";

//   transformStamped3.transform.translation.x = 0;
//   transformStamped3.transform.translation.y = 0;
//   transformStamped3.transform.translation.z = wheel_radius;

//   transformStamped3.transform.rotation.x = Quaternion3.getX();
//   transformStamped3.transform.rotation.y = Quaternion3.getY();
//   transformStamped3.transform.rotation.z = Quaternion3.getZ();
//   transformStamped3.transform.rotation.w = Quaternion3.getW();

//   Quaternion4.setRPY(0, 0, 0);
//   transformStamped4.header.frame_id = "base_link";
//   transformStamped4.child_frame_id = "right_wheel";

//   transformStamped4.transform.translation.x = 0;
//   transformStamped4.transform.translation.y = -turning_radius;
//   transformStamped4.transform.translation.z = 0;

//   transformStamped4.transform.rotation.x = Quaternion4.getX();
//   transformStamped4.transform.rotation.y = Quaternion4.getY();
//   transformStamped4.transform.rotation.z = Quaternion4.getZ();
//   transformStamped4.transform.rotation.w = Quaternion4.getW();

//   Quaternion5.setRPY(0, 0, 0);
//   transformStamped5.header.frame_id = "base_link";
//   transformStamped5.child_frame_id = "left_wheel";

//   transformStamped5.transform.translation.x = 0;
//   transformStamped5.transform.translation.y = turning_radius;
//   transformStamped5.transform.translation.z = 0;

//   transformStamped5.transform.rotation.x = Quaternion5.getX();
//   transformStamped5.transform.rotation.y = Quaternion5.getY();
//   transformStamped5.transform.rotation.z = Quaternion5.getZ();
//   transformStamped5.transform.rotation.w = Quaternion5.getW();

//   Quaternion6.setRPY(0, 0, 0);
//   transformStamped6.header.frame_id = "base_link";
//   transformStamped6.child_frame_id = "steer_wheel";

//   transformStamped6.transform.translation.x = steer_wheel_x;
//   transformStamped6.transform.translation.y = steer_wheel_y;
//   transformStamped6.transform.translation.z = 0;

//   transformStamped6.transform.rotation.x = Quaternion6.getX();
//   transformStamped6.transform.rotation.y = Quaternion6.getY();
//   transformStamped6.transform.rotation.z = Quaternion6.getZ();
//   transformStamped6.transform.rotation.w = Quaternion6.getW();

//   Quaternion7.setRPY(0, 0, laser_base_angle_yaw);
//   transformStamped7.header.stamp = ros::Time::now();
//   transformStamped7.header.frame_id = "nav_footprint";
//   transformStamped7.child_frame_id = "laser_map";

//   transformStamped7.transform.translation.x = 0;
//   transformStamped7.transform.translation.y = 0;
//   transformStamped7.transform.translation.z = laser_base_z;

//   transformStamped7.transform.rotation.x = Quaternion7.getX();
//   transformStamped7.transform.rotation.y = Quaternion7.getY();
//   transformStamped7.transform.rotation.z = Quaternion7.getZ();
//   transformStamped7.transform.rotation.w = Quaternion7.getW();

//   Quaternion9.setRPY(0, 0, laser_1_angle_yaw);
//   transformStamped9.header.frame_id = "base_link";
//   transformStamped9.child_frame_id = "laser_1";

//   transformStamped9.transform.translation.x = laser_1_x;
//   transformStamped9.transform.translation.y = laser_1_y;
//   transformStamped9.transform.translation.z = 0;

//   transformStamped9.transform.rotation.x = Quaternion9.getX();
//   transformStamped9.transform.rotation.y = Quaternion9.getY();
//   transformStamped9.transform.rotation.z = Quaternion9.getZ();
//   transformStamped9.transform.rotation.w = Quaternion9.getW();

//   Quaternion10.setRPY(0, 0, laser_2_angle_yaw);
//   transformStamped10.header.frame_id = "base_link";
//   transformStamped10.child_frame_id = "laser_2";

//   transformStamped10.transform.translation.x = laser_2_x;
//   transformStamped10.transform.translation.y = laser_2_y;
//   transformStamped10.transform.translation.z = 0;

//   transformStamped10.transform.rotation.x = Quaternion10.getX();
//   transformStamped10.transform.rotation.y = Quaternion10.getY();
//   transformStamped10.transform.rotation.z = Quaternion10.getZ();
//   transformStamped10.transform.rotation.w = Quaternion10.getW();

//   while (ros::ok())
//   {
//     ros::spinOnce();
//     if (nav_status)
//     {
//       transformStamped8.header.stamp = ros::Time::now();
//       transformStamped8.header.frame_id = "odom";
//       transformStamped8.child_frame_id = "nav_footprint";

//       transformStamped8.transform.translation.x = odom_nav.pose.pose.position.x;
//       transformStamped8.transform.translation.y = odom_nav.pose.pose.position.y;
//       transformStamped8.transform.translation.z = odom_nav.pose.pose.position.z;

//       transformStamped8.transform.rotation.x = odom_nav.pose.pose.orientation.x;
//       transformStamped8.transform.rotation.y = odom_nav.pose.pose.orientation.y;
//       transformStamped8.transform.rotation.z = odom_nav.pose.pose.orientation.z;
//       transformStamped8.transform.rotation.w = odom_nav.pose.pose.orientation.w;
//       br8.sendTransform(transformStamped8);
//     }
//     transformStamped1.header.stamp = ros::Time::now();
//     transformStamped2.header.stamp = ros::Time::now();
//     transformStamped3.header.stamp = ros::Time::now();
//     transformStamped4.header.stamp = ros::Time::now();
//     transformStamped5.header.stamp = ros::Time::now();
//     transformStamped6.header.stamp = ros::Time::now();
//     transformStamped7.header.stamp = ros::Time::now();
//     transformStamped9.header.stamp = ros::Time::now();
//     transformStamped10.header.stamp = ros::Time::now();

//     br1.sendTransform(transformStamped1);
//     br2.sendTransform(transformStamped2);
//     br3.sendTransform(transformStamped3);
//     br4.sendTransform(transformStamped4);
//     br5.sendTransform(transformStamped5);
//     br6.sendTransform(transformStamped6);
//     br7.sendTransform(transformStamped7);
//     // br9.sendTransform(transformStamped9);
//     // br10.sendTransform(transformStamped10);
//     ros::Duration(0.05).sleep();
//   }
// }






// #include <ros/ros.h>

// #include <tf2_geometry_msgs/tf2_geometry_msgs.h>
// #include <tf2_ros/transform_broadcaster.h>
// #include <tf2_ros/transform_listener.h>

// #include <geometry_msgs/TransformStamped.h>
// #include <nav_msgs/Odometry.h>

// nav_msgs::Odometry odom_nav;
// bool nav_status = false;

// void laserCallback(const nav_msgs::Odometry::ConstPtr& msg)
// {
//   odom_nav = *msg;
//   nav_status = true;
// }

// int main(int argc, char** argv)
// {
//   ros::init(argc, argv, "forklift_tf_publisher");
//   ros::NodeHandle nh;
//   ros::NodeHandle nh_priv("~");

//   double map_odom_angle, laser_base_x, laser_base_y, laser_base_z, laser_base_angle_yaw,
//       wheel_radius, turning_radius, steer_wheel_x, steer_wheel_y, laser_1_x, laser_1_y, laser_1_angle_yaw,
//       laser_2_x, laser_2_y, laser_2_angle_yaw;
//   nh_priv.param<double>("map_odom_angle", map_odom_angle, 0.0);
//   nh_priv.param<double>("laser_base_x", laser_base_x, 0.0);
//   nh_priv.param<double>("laser_base_y", laser_base_y, 0.0);
//   nh_priv.param<double>("laser_base_z", laser_base_z, 0.0);
//   nh_priv.param<double>("laser_base_angle_yaw", laser_base_angle_yaw, 0.0);
//   nh_priv.param<double>("wheel_radius", wheel_radius, 0.0);
//   nh_priv.param<double>("turning_radius", turning_radius, 0.0);
//   nh_priv.param<double>("steer_wheel_x", steer_wheel_x, 0.0);
//   nh_priv.param<double>("steer_wheel_y", steer_wheel_y, 0.0);
//   nh_priv.param<double>("laser_1_x", laser_1_x, 0.0);
//   nh_priv.param<double>("laser_1_y", laser_1_y, 0.0);
//   nh_priv.param<double>("laser_1_angle_yaw", laser_1_angle_yaw, 0.0);
//   nh_priv.param<double>("laser_2_x", laser_2_x, 0.0);
//   nh_priv.param<double>("laser_2_y", laser_2_y, 0.0);
//   nh_priv.param<double>("laser_2_angle_yaw", laser_2_angle_yaw, 0.0);

//   ros::Subscriber sub_nav = nh.subscribe("/nav350laser/odom", 1, laserCallback);

//   static tf2_ros::TransformBroadcaster br1;
//   static tf2_ros::TransformBroadcaster br2;
//   static tf2_ros::TransformBroadcaster br3;
//   static tf2_ros::TransformBroadcaster br4;
//   static tf2_ros::TransformBroadcaster br5;
//   static tf2_ros::TransformBroadcaster br6;
//   static tf2_ros::TransformBroadcaster br7;
//   static tf2_ros::TransformBroadcaster br8;
//   static tf2_ros::TransformBroadcaster br9;
//   static tf2_ros::TransformBroadcaster br10;

//   tf2::Quaternion Quaternion1;
//   tf2::Quaternion Quaternion2;
//   tf2::Quaternion Quaternion3;
//   tf2::Quaternion Quaternion4;
//   tf2::Quaternion Quaternion5;
//   tf2::Quaternion Quaternion6;
//   tf2::Quaternion Quaternion7;
//   tf2::Quaternion Quaternion8;
//   tf2::Quaternion Quaternion9;
//   tf2::Quaternion Quaternion10;

//   geometry_msgs::TransformStamped transformStamped1;
//   geometry_msgs::TransformStamped transformStamped2;
//   geometry_msgs::TransformStamped transformStamped3;
//   geometry_msgs::TransformStamped transformStamped4;
//   geometry_msgs::TransformStamped transformStamped5;
//   geometry_msgs::TransformStamped transformStamped6;
//   geometry_msgs::TransformStamped transformStamped7;
//   geometry_msgs::TransformStamped transformStamped8;
//   geometry_msgs::TransformStamped transformStamped9;
//   geometry_msgs::TransformStamped transformStamped10;

//   Quaternion1.setRPY(0, 0, map_odom_angle);
//   transformStamped1.header.frame_id = "map";
//   transformStamped1.child_frame_id = "odom";

//   transformStamped1.transform.translation.x = 0;
//   transformStamped1.transform.translation.y = 0;
//   transformStamped1.transform.translation.z = 0;

//   transformStamped1.transform.rotation.x = Quaternion1.getX();
//   transformStamped1.transform.rotation.y = Quaternion1.getY();
//   transformStamped1.transform.rotation.z = Quaternion1.getZ();
//   transformStamped1.transform.rotation.w = Quaternion1.getW();

//   Quaternion2.setRPY(0, 0, laser_base_angle_yaw);
//   transformStamped2.header.frame_id = "nav_footprint";
//   transformStamped2.child_frame_id = "base_footprint";

//   transformStamped2.transform.translation.x = laser_base_x;
//   transformStamped2.transform.translation.y = laser_base_y;
//   transformStamped2.transform.translation.z = 0;

//   transformStamped2.transform.rotation.x = Quaternion2.getX();
//   transformStamped2.transform.rotation.y = Quaternion2.getY();
//   transformStamped2.transform.rotation.z = Quaternion2.getZ();
//   transformStamped2.transform.rotation.w = Quaternion2.getW();

//   Quaternion3.setRPY(0, 0, 0);
//   transformStamped3.header.frame_id = "base_footprint";
//   transformStamped3.child_frame_id = "base_link";

//   transformStamped3.transform.translation.x = 0;
//   transformStamped3.transform.translation.y = 0;
//   transformStamped3.transform.translation.z = wheel_radius;

//   transformStamped3.transform.rotation.x = Quaternion3.getX();
//   transformStamped3.transform.rotation.y = Quaternion3.getY();
//   transformStamped3.transform.rotation.z = Quaternion3.getZ();
//   transformStamped3.transform.rotation.w = Quaternion3.getW();

//   Quaternion4.setRPY(0, 0, 0);
//   transformStamped4.header.frame_id = "base_link";
//   transformStamped4.child_frame_id = "right_wheel";

//   transformStamped4.transform.translation.x = 0;
//   transformStamped4.transform.translation.y = -turning_radius;
//   transformStamped4.transform.translation.z = 0;

//   transformStamped4.transform.rotation.x = Quaternion4.getX();
//   transformStamped4.transform.rotation.y = Quaternion4.getY();
//   transformStamped4.transform.rotation.z = Quaternion4.getZ();
//   transformStamped4.transform.rotation.w = Quaternion4.getW();

//   Quaternion5.setRPY(0, 0, 0);
//   transformStamped5.header.frame_id = "base_link";
//   transformStamped5.child_frame_id = "left_wheel";

//   transformStamped5.transform.translation.x = 0;
//   transformStamped5.transform.translation.y = turning_radius;
//   transformStamped5.transform.translation.z = 0;

//   transformStamped5.transform.rotation.x = Quaternion5.getX();
//   transformStamped5.transform.rotation.y = Quaternion5.getY();
//   transformStamped5.transform.rotation.z = Quaternion5.getZ();
//   transformStamped5.transform.rotation.w = Quaternion5.getW();

//   Quaternion6.setRPY(0, 0, 0);
//   transformStamped6.header.frame_id = "base_link";
//   transformStamped6.child_frame_id = "steer_wheel";

//   transformStamped6.transform.translation.x = steer_wheel_x;
//   transformStamped6.transform.translation.y = steer_wheel_y;
//   transformStamped6.transform.translation.z = 0;

//   transformStamped6.transform.rotation.x = Quaternion6.getX();
//   transformStamped6.transform.rotation.y = Quaternion6.getY();
//   transformStamped6.transform.rotation.z = Quaternion6.getZ();
//   transformStamped6.transform.rotation.w = Quaternion6.getW();

//   Quaternion7.setRPY(0, 0, laser_base_angle_yaw);
//   transformStamped7.header.stamp = ros::Time::now();
//   transformStamped7.header.frame_id = "nav_footprint";
//   transformStamped7.child_frame_id = "laser_map";

//   transformStamped7.transform.translation.x = 0;
//   transformStamped7.transform.translation.y = 0;
//   transformStamped7.transform.translation.z = laser_base_z;

//   transformStamped7.transform.rotation.x = Quaternion7.getX();
//   transformStamped7.transform.rotation.y = Quaternion7.getY();
//   transformStamped7.transform.rotation.z = Quaternion7.getZ();
//   transformStamped7.transform.rotation.w = Quaternion7.getW();

//   Quaternion9.setRPY(0, 0, laser_1_angle_yaw);
//   transformStamped9.header.frame_id = "base_link";
//   transformStamped9.child_frame_id = "laser_1";

//   transformStamped9.transform.translation.x = laser_1_x;
//   transformStamped9.transform.translation.y = laser_1_y;
//   transformStamped9.transform.translation.z = 0;

//   transformStamped9.transform.rotation.x = Quaternion9.getX();
//   transformStamped9.transform.rotation.y = Quaternion9.getY();
//   transformStamped9.transform.rotation.z = Quaternion9.getZ();
//   transformStamped9.transform.rotation.w = Quaternion9.getW();

//   Quaternion10.setRPY(0, 0, laser_2_angle_yaw);
//   transformStamped10.header.frame_id = "base_link";
//   transformStamped10.child_frame_id = "laser_2";

//   transformStamped10.transform.translation.x = laser_2_x;
//   transformStamped10.transform.translation.y = laser_2_y;
//   transformStamped10.transform.translation.z = 0;

//   transformStamped10.transform.rotation.x = Quaternion10.getX();
//   transformStamped10.transform.rotation.y = Quaternion10.getY();
//   transformStamped10.transform.rotation.z = Quaternion10.getZ();
//   transformStamped10.transform.rotation.w = Quaternion10.getW();

//   while (ros::ok())
//   {
//     ros::spinOnce();
//     if (nav_status)
//     {
//       transformStamped8.header.stamp = ros::Time::now();
//       transformStamped8.header.frame_id = "odom";
//       transformStamped8.child_frame_id = "nav_footprint";

//       transformStamped8.transform.translation.x = odom_nav.pose.pose.position.x;
//       transformStamped8.transform.translation.y = odom_nav.pose.pose.position.y;
//       transformStamped8.transform.translation.z = odom_nav.pose.pose.position.z;

//       transformStamped8.transform.rotation.x = odom_nav.pose.pose.orientation.x;
//       transformStamped8.transform.rotation.y = odom_nav.pose.pose.orientation.y;
//       transformStamped8.transform.rotation.z = odom_nav.pose.pose.orientation.z;
//       transformStamped8.transform.rotation.w = odom_nav.pose.pose.orientation.w;
//       br8.sendTransform(transformStamped8);
//     }
//     transformStamped1.header.stamp = ros::Time::now();
//     transformStamped2.header.stamp = ros::Time::now();
//     transformStamped3.header.stamp = ros::Time::now();
//     transformStamped4.header.stamp = ros::Time::now();
//     transformStamped5.header.stamp = ros::Time::now();
//     transformStamped6.header.stamp = ros::Time::now();
//     transformStamped7.header.stamp = ros::Time::now();
//     transformStamped9.header.stamp = ros::Time::now();
//     transformStamped10.header.stamp = ros::Time::now();

//     br1.sendTransform(transformStamped1);
//     br2.sendTransform(transformStamped2);
//     br3.sendTransform(transformStamped3);
//     br4.sendTransform(transformStamped4);
//     br5.sendTransform(transformStamped5);
//     br6.sendTransform(transformStamped6);
//     br7.sendTransform(transformStamped7);
//     // br9.sendTransform(transformStamped9);
//     // br10.sendTransform(transformStamped10);
//     ros::Duration(0.05).sleep();
//   }
// }