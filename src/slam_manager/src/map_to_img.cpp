#include "ros/ros.h"
#include <std_msgs/Int16.h>
#include <MapAsImageProvider.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

using namespace std;

MapAsImageProvider *map_image_provider;
int32_t tile_width;
int32_t tile_height;
std::string parent_frame;
std::string child_frame;
bool draw_robot;
bool publish_full_map;
bool publish_map_tile;

void map_zoom_callback(const std_msgs::Int16 &scale)
{
  map_image_provider->setScale((float)scale.data / 2500);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "map_to_image_node");
  ros::NodeHandle n("~");

  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener(tf_buffer);

  n.param<int32_t>("tile_width", tile_width, INITIAL_TILE_SIZE_X);
  n.param<int32_t>("tile_height", tile_height, INITIAL_TILE_SIZE_Y);
  n.param<std::string>("parent_frame", parent_frame, "map");
  n.param<std::string>("child_frame", child_frame, "base_link");
  n.param<bool>("draw_robot", draw_robot, false);
  n.param<bool>("publish_full_map", publish_full_map, true);
  n.param<bool>("publish_map_tile", publish_map_tile, true);

  ros::Rate loop_rate(1);
  ROS_INFO("Init MapAsImageProvider object");
  map_image_provider = new MapAsImageProvider(n, tile_width, tile_height, draw_robot, publish_full_map, publish_map_tile);
  ros::Subscriber map_zoom_sub = n.subscribe("/map_zoom", 1, map_zoom_callback);

  while (ros::ok())
  {
    ros::spinOnce();

    try
    {
      geometry_msgs::TransformStamped transformStamped = tf_buffer.lookupTransform(parent_frame, child_frame, ros::Time(0));
      tf2::Transform tf_transform;
      tf2::fromMsg(transformStamped.transform, tf_transform);

      tf2::Matrix3x3 robot_rot_matrix(tf_transform.getRotation());
      double yaw, pitch, roll;
      robot_rot_matrix.getEulerYPR(yaw, pitch, roll);

      map_image_provider->updateRobotPosition(tf_transform.getOrigin().getX(), tf_transform.getOrigin().getY(), yaw);
    }
    catch (tf2::TransformException &ex)
    {
      ROS_WARN("Could not get transform between '%s' and '%s', will retry every second", parent_frame.c_str(), child_frame.c_str());
      ros::Duration(1.0).sleep();
    }

    loop_rate.sleep();
    if (publish_full_map)
    {
      map_image_provider->publishFullMap();
    }

    if (publish_map_tile)
    {
      map_image_provider->publishMapTile();
    }
  }

  delete map_image_provider; // Clean up allocated memory
  return 0;
}
