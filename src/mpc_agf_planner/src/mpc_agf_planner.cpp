#include <mpc_agf_planner/mpc_agf_planner.h>

#include <tf2/utils.h>

#include <geometry_msgs/PoseStamped.h>

#include <base_local_planner/footprint_helper.h>
#include <base_local_planner/goal_functions.h>
#include <pluginlib/class_list_macros.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <mbf_msgs/ExePathResult.h>

#include <agv_common_library/get_hardware_hash.h>
#include <math.h>

PLUGINLIB_EXPORT_CLASS(mpc_agf_planner::MpcAgfPlannerROS,
                       nav_core::BaseLocalPlanner)
PLUGINLIB_EXPORT_CLASS(mpc_agf_planner::MpcAgfPlannerROS,
                       mbf_costmap_core::CostmapController)

namespace mpc_agf_planner
{
using json = nlohmann::json;
MpcAgfPlannerROS::MpcAgfPlannerROS()
    : costmap_ros_(NULL), tf_(NULL), costmap_model_(NULL), goal_reached_(false),
      initialized_(false), dynamic_recfg_(NULL)
{
}

MpcAgfPlannerROS::~MpcAgfPlannerROS() {}

void MpcAgfPlannerROS::initialize(std::string name, tf2_ros::Buffer* tf,
                  costmap_2d::Costmap2DROS* costmap_ros)
{
  // License verification (commented out)
  // if (!getUbuntuSerialHash())
  // {
  //   ROS_ERROR("MpcAgfPlanner: License verification failed.");
  //   return;
  // }

  // ============================================================================
  // BASIC INITIALIZATION
  // ============================================================================
  name_ = name;
  ros::NodeHandle private_nh("~/" + name);
  tf_ = tf;
  costmap_ros_ = costmap_ros;
  costmap_ = costmap_ros_->getCostmap();
  global_frame_ = costmap_ros_->getGlobalFrameID();
  robot_base_frame_ = costmap_ros_->getBaseFrameID();

  // ============================================================================
  // LOAD PARAMETERS FROM ROS PARAMETER SERVER
  // ============================================================================

  // Odometry configuration
  odom_topic_ = private_nh.param<std::string>("odom_topic", "/odom");
  odom_helper_.setOdomTopic(odom_topic_);

  // Velocity limits
  max_vel_x_ = private_nh.param<double>("max_vel_x", 0.7);
  max_vel_requested = abs(max_vel_x_);
  max_angular_z_ = private_nh.param<double>("max_angular_z", 1.5);
  vel_x_when_more_angle_ = private_nh.param<double>("vel_x_when_more_angle", 0.25);
  init_vel_x_when_more_angle_ = vel_x_when_more_angle_;

  // MPC path tracking: model (from script/identify_model.py), weights, speed profile
  ref_point_offset_ = private_nh.param<double>("ref_point_offset", 0.0);
  path_ds_ = private_nh.param<double>("path_ds", 0.05);
  kappa_half_span_ = private_nh.param<double>("kappa_half_span", 0.25);
  a_acc_ = private_nh.param<double>("a_acc", 0.4);
  a_brake_ = private_nh.param<double>("a_brake", 1.2);
  v_min_mpc_ = private_nh.param<double>("v_min_mpc", 0.05);
  speed_limits_.v_max = max_vel_x_;
  speed_limits_.a_lat_max = private_nh.param<double>("a_lat_max", 0.25);
  speed_limits_.a_dec = private_nh.param<double>("a_dec", 0.3);
  speed_limits_.v_creep = private_nh.param<double>("v_creep_min", 0.1);
  // Read here so xy_goal_tolerance is known: creep from creep_dist before the tolerance zone
  double creep_dist = private_nh.param<double>("creep_dist", 0.2);
  speed_limits_.creep_dist = private_nh.param<double>("xy_goal_tolerance", 0.1) + creep_dist;
  stop_lead_time_ = private_nh.param<double>("stop_lead_time", 0.5);
  // Reverse stopping: the robot coasts/brakes differently backwards, so these
  // can be set on their own; unset (< 0) = same as forward
  a_dec_reverse_cfg_ = private_nh.param<double>("a_dec_reverse", -1.0);
  v_creep_reverse_cfg_ = private_nh.param<double>("v_creep_min_reverse", -1.0);
  creep_dist_reverse_cfg_ = private_nh.param<double>("creep_dist_reverse", -1.0);
  stop_lead_time_reverse_cfg_ = private_nh.param<double>("stop_lead_time_reverse", -1.0);
  xy_goal_tolerance_ = private_nh.param<double>("xy_goal_tolerance", 0.1);  // used below
  updateReverseLimits();
  // /steering_angle and angular.z are the LEFT wheel's angle (Ackermann);
  // the MPC works with the virtual centre angle
  MpcParams mpc_params = loadMpcParams(private_nh);
  steer_geom_.wheelbase = mpc_params.wheelbase;
  steer_geom_.steer_sign = mpc_params.steer_sign;
  steer_geom_.track_width = private_nh.param<double>(
      "steer_track_width", private_nh.param<double>("track_width", 0.0));
  if (steer_geom_.track_width <= 0.0)
  {
    ROS_WARN("steer_track_width not set: treating /steering_angle as the centre angle");
  }
  mpc_.setParams(mpc_params);  // max_steer is set below, once the wheel limits are known
  base_q_epsi_ = mpc_params.q_epsi;
  max_correction_attempts_ = private_nh.param<int>("max_correction_attempts", 2);

  // Slow down while the wheel cannot follow the command (0 = off)
  steering_lag_slowdown_ = private_nh.param<double>("steering_lag_slowdown", 0.15);
  steering_lagging_ = false;
  priority_angle_adjustment_ = false;
  heading_priority_active_ = false;

  // Creep after the stop command: the robot keeps its speed for brake_delay,
  // then decelerates at brake_decel. Stop is commanded that far before goal.
  // Both 0: stop_lead_time is used instead.
  brake_delay_ = private_nh.param<double>("brake_delay", 0.0);
  brake_decel_ = private_nh.param<double>("brake_decel", 0.0);
  odom_timeout_ = private_nh.param<double>("odom_timeout", 0.5);

  // Goal tolerance parameters
  error_to_start_stop_ = private_nh.param<double>("error_to_start_stop", 0.3);
  xy_goal_tolerance_ = private_nh.param<double>("xy_goal_tolerance", 0.1);
  yaw_goal_tolerance_ = private_nh.param<double>("yaw_goal_tolerance", 0.1);
  ignore_last_rotate_ = private_nh.param<bool>("ignore_last_rotate", true);

  // Waypoint tolerance parameters
  error_to_low_speed_points_ = private_nh.param<double>("error_to_low_speed_points", 1.5);
  error_to_special_safety_points_ = private_nh.param<double>("error_to_special_safety_points", 0.5);

  // Car-like robot specific parameters
  is_carlike_ = private_nh.param<bool>("is_carlike", true);
  steering_angle_for_rotate_ = private_nh.param<double>("car_like/steering_angle_for_rotate", 1.57);
  steering_angle_for_rotate_min_ = private_nh.param<double>("car_like/steering_angle_for_rotate_min", -steering_angle_for_rotate_);
  steering_angle_for_rotate_max_ = private_nh.param<double>("car_like/steering_angle_for_rotate_max", steering_angle_for_rotate_);
  steering_angle_tolerance_ = private_nh.param<double>("car_like/steering_angle_tolerance", 0.05);
  max_trans_vel_for_rotate_ = private_nh.param<double>("car_like/max_trans_vel_for_rotate", 0.8);

  // Unused/legacy parameters (kept for compatibility)
  angle_to_global_when_start_ = private_nh.param<double>("angle_to_global_when_start", 0.05);
  error_angle_stop_ = private_nh.param<double>("error_angle_stop", 0.1);
  max_error_pose_to_path_ = private_nh.param<double>("max_error_pose_to_path", 0.2);

  // Steering limits of the left wheel (configured in degrees), converted to
  // center angle for the MPC
  min_steering_angle_ = std::max(-max_angular_z_,
      private_nh.param<double>("min_steering_angle_deg", steering_angle_for_rotate_min_ * 180.0 / M_PI) * M_PI / 180.0);
  max_steering_angle_ = std::min(max_angular_z_,
      private_nh.param<double>("max_steering_angle_deg", steering_angle_for_rotate_max_ * 180.0 / M_PI) * M_PI / 180.0);
  // The MPC has one symmetric limit, use the wider side; the command is clamped
  // to each side of the left wheel in trackPath()
  MpcParams limited = mpc_.params();
  limited.max_steer = std::max(fabs(steer_geom_.leftToCentre(min_steering_angle_)),
                               fabs(steer_geom_.leftToCentre(max_steering_angle_)));
  // The model uses tan(delta), which flips sign at 90 degrees
  const double max_centre = 1.4;
  if (limited.max_steer > max_centre)
  {
    ROS_WARN("Steering limits give a centre angle of %.2f rad; limiting the MPC to %.2f rad. "
             "Set them to the wheel's real limit", limited.max_steer, max_centre);
    limited.max_steer = max_centre;
  }
  mpc_.setParams(limited);
  ROS_INFO("MPC steering: L %.3f m, steer_sign %.0f, track width %.3f m, left wheel [%.3f, %.3f] rad, centre limit %.3f rad",
           limited.wheelbase, limited.steer_sign, steer_geom_.track_width,
           min_steering_angle_, max_steering_angle_, limited.max_steer);

  // State flags
  rotate_to_goal_ = private_nh.param<bool>("rotate_to_goal", false);
  allow_set_safety_ = true;
  correction_attempts_ = 0;
  path_hint_ = -1;
  last_steer_cmd_ = 0.0;
  last_vel_cmd_ = 0.0;
  steering_angle_ = 0.0;

  // ============================================================================
  // SETUP PUBLISHERS
  // ============================================================================
  pub_path_ = nh_.advertise<nav_msgs::Path>("/move_base/GlobalPlanner/plan", 1);
  pub_safety_control_ = nh_.advertise<agf_msgs::Int8Array>("/safety_control", 1);

  // Visualization publishers for debugging path following
  pub_local_path_ = nh_.advertise<nav_msgs::Path>("/mpc_agf_planner/local_path", 1);
  pub_mpc_predicted_ = nh_.advertise<nav_msgs::Path>("/mpc_agf_planner/mpc_prediction", 1);
  pub_reference_point_ = nh_.advertise<visualization_msgs::Marker>("/mpc_agf_planner/nearest_point", 1);
  pub_tracking_error_ = nh_.advertise<std_msgs::Float64MultiArray>("/mpc_agf_planner/tracking_error", 1);

  // ============================================================================
  // SETUP SUBSCRIBERS
  // ============================================================================
  sub_cmd_vel_ = nh_.subscribe("/cmd_vel_feedback", 1, &MpcAgfPlannerROS::cmdVelCallback, this);
  // sub_safety_zone_ = nh_.subscribe("/safety_status", 1, &MpcAgfPlannerROS::safetyCallback, this);
  sub_angle_steer_ = nh_.subscribe("/steering_angle", 1, &MpcAgfPlannerROS::steeringAngleCallback, this);
  request_speed_limit_sub = nh_.subscribe("set_safety/request_zone_speed_limit", 1, &MpcAgfPlannerROS::request_speed_limit_cb, this);


  // ============================================================================
  // SETUP SERVICE CLIENTS
  // ============================================================================
  client_get_waypoints_low_speed_ = nh_.serviceClient<agv_msgs::ArrayWaypoints>("/waypoints_low_speed");
  client_get_waypoints_special_safety_ = nh_.serviceClient<agv_msgs::ArraySafetyWaypoints>("/waypoints_special_safety");

  // ============================================================================
  // SETUP DYNAMIC RECONFIGURE
  // ============================================================================
  dynamic_recfg_ = boost::make_shared<dynamic_reconfigure::Server<MpcAgfPlannerReconfigureConfig>>(private_nh);
  dynamic_reconfigure::Server<MpcAgfPlannerReconfigureConfig>::CallbackType cb =
    boost::bind(&MpcAgfPlannerROS::reconfigureCB, this, _1, _2);
  dynamic_recfg_->setCallback(cb);

  // ============================================================================
  // FINAL INITIALIZATION
  // ============================================================================
  msg_safety_.ints.resize(4);
  old_time_ = ros::Time::now().toSec();

  std::cout << "vel_x_when_more_angle: " << vel_x_when_more_angle_ << std::endl;

  initialized_ = true;
}

void MpcAgfPlannerROS::reconfigureCB(MpcAgfPlannerReconfigureConfig& config,
                                       uint32_t level)
{
  if (!initialized_)
  {
    return;
  }
  allow_set_safety_ = config.allow_set_safety;
  priority_angle_adjustment_ = config.priority_angle_adjustment;
  ROS_ERROR("Set allow_set_safety: %d", allow_set_safety_);

  // Live MPC tuning; the speed profile is rebuilt on the next setPlan()
  std::lock_guard<std::mutex> lock(params_mutex_);
  MpcParams p = mpc_.params();
  base_q_epsi_ = config.q_epsi;
  heading_priority_active_ = false;
  p.q_ey = config.q_ey;
  p.q_epsi = config.q_epsi;
  p.r_steer = config.r_steer;
  p.r_dsteer = config.r_dsteer;
  p.qf_scale = config.qf_scale;
  mpc_.setParams(p);
  speed_limits_.a_lat_max = config.a_lat_max;
  speed_limits_.a_dec = config.a_dec;
  speed_limits_.v_creep = config.v_creep_min;
  speed_limits_.creep_dist = xy_goal_tolerance_ + config.creep_dist;
  stop_lead_time_ = config.stop_lead_time;
  a_dec_reverse_cfg_ = config.a_dec_reverse;
  v_creep_reverse_cfg_ = config.v_creep_min_reverse;
  creep_dist_reverse_cfg_ = config.creep_dist_reverse;
  stop_lead_time_reverse_cfg_ = config.stop_lead_time_reverse;
  updateReverseLimits();
  ROS_INFO("MPC weights: q_ey %.2f q_epsi %.2f r_steer %.2f r_dsteer %.2f qf %.1f",
           p.q_ey, p.q_epsi, p.r_steer, p.r_dsteer, p.qf_scale);
}

void MpcAgfPlannerROS::updateReverseLimits()
{
  // speed_limits_.creep_dist already includes xy_goal_tolerance
  speed_limits_reverse_ = speed_limits_;
  if (a_dec_reverse_cfg_ >= 0.0)
  {
    speed_limits_reverse_.a_dec = std::max(0.01, a_dec_reverse_cfg_);
  }
  if (v_creep_reverse_cfg_ >= 0.0)
  {
    speed_limits_reverse_.v_creep = v_creep_reverse_cfg_;
  }
  if (creep_dist_reverse_cfg_ >= 0.0)
  {
    speed_limits_reverse_.creep_dist = xy_goal_tolerance_ + creep_dist_reverse_cfg_;
  }
  stop_lead_time_reverse_ =
      stop_lead_time_reverse_cfg_ >= 0.0 ? stop_lead_time_reverse_cfg_ : stop_lead_time_;
  ROS_INFO("Reverse stop: a_dec %.2f m/s^2, creep %.2f m/s over %.2f m, stop lead %.2f s",
           speed_limits_reverse_.a_dec, speed_limits_reverse_.v_creep,
           speed_limits_reverse_.creep_dist, stop_lead_time_reverse_);
}

MpcParams MpcAgfPlannerROS::loadMpcParams(ros::NodeHandle& nh) const
{
  MpcParams p;
  if (nh.hasParam("wheelbase"))
  {
    p.wheelbase = nh.param<double>("wheelbase", p.wheelbase);
    p.steer_sign = nh.param<double>("steer_sign", p.steer_sign) < 0.0 ? -1.0 : 1.0;
  }
  else
  {
    // Signed x of the steering axle in base_link (center of fixed axle), < 0 when
    // the steering axle is behind base_link (rear steering)
    double steer_axle_x = nh.param<double>("steer_axle_x", -1.25);
    if (fabs(steer_axle_x) < 0.1)
    {
      ROS_ERROR("steer_axle_x %.3f m is invalid, using -1.25 m", steer_axle_x);
      steer_axle_x = -1.25;
    }
    p.wheelbase = fabs(steer_axle_x);
    p.steer_sign = steer_axle_x < 0.0 ? -1.0 : 1.0;
  }
  p.tau_steer = nh.param<double>("tau_steer", p.tau_steer);
  p.max_steer = max_angular_z_;  // replaced by the centre-angle limit in initialize()
  p.max_steer_rate = nh.param<double>(
      "max_steer_rate", nh.param<double>("max_steering_rate", p.max_steer_rate));
  if (p.max_steer_rate <= 0.0)
  {
    p.max_steer_rate = 10.0;  // Used to mean no limit
  }
  p.delay = nh.param<double>("delay", p.delay);
  p.dt = nh.param<double>("mpc_dt", p.dt);
  p.horizon = nh.param<int>("mpc_horizon", p.horizon);
  p.q_ey = nh.param<double>("q_ey", p.q_ey);
  p.q_epsi = nh.param<double>("q_epsi", p.q_epsi);
  p.r_steer = nh.param<double>("r_steer", p.r_steer);
  p.r_dsteer = nh.param<double>("r_dsteer", p.r_dsteer);
  p.qf_scale = nh.param<double>("qf_scale", p.qf_scale);
  ROS_INFO("MPC model: L %.3f m, steer_sign %.0f, tau %.3f s, rate %.2f rad/s, delay %.3f s",
           p.wheelbase, p.steer_sign, p.tau_steer, p.max_steer_rate, p.delay);
  return p;
}

void MpcAgfPlannerROS::cmdVelCallback(
    const geometry_msgs::Twist::ConstPtr msg)
{
  robot_vel_ = *msg;
}

void MpcAgfPlannerROS::steeringAngleCallback(
    const std_msgs::Float64::ConstPtr msg)
{
  steering_angle_ = msg->data;
}

void MpcAgfPlannerROS::request_speed_limit_cb(
  std_stamped_msgs::StringStamped::ConstPtr msg)
{
  try {
    // Parse simple float value from string
    max_vel_requested = abs(std::stod(msg->data));
    ROS_INFO_ONCE("Set max_vel_x: %.2f m/s", max_vel_requested);
  }
  catch (const std::exception& e) {
    ROS_ERROR_ONCE("Failed to parse speed limit: %s", e.what());
    max_vel_requested = abs(max_vel_x_);
    ROS_WARN_ONCE("Max_vel set to default: %f", max_vel_requested);
  }
}

bool MpcAgfPlannerROS::setPlan(
    const std::vector<geometry_msgs::PoseStamped>& orig_global_plan)
{
  ROS_INFO("-----Local planner: begin set plan-----");
  if (!initialized_)
  {
    ROS_ERROR("This planner has not been initialized, please call initialize() "
              "before using this planner");
    return false;
  }

  if (orig_global_plan.empty())
  {
    ROS_ERROR("Local planner: received an empty plan");
    return false;
  }
  global_goal_ = orig_global_plan.back();

  // Get status robot go forward or backward
  if (orig_global_plan.front().pose.orientation.x == 1)
  {
    go_forward_ = false;
  }
  else
  {
    go_forward_ = true;
  }

  // Build the MPC reference: resampled path, curvature and speed profile
  geometry_msgs::PoseStamped robot_pose;
  costmap_ros_->getRobotPose(robot_pose);
  double yaw_robot = quaternionToRPY(
      robot_pose.pose.orientation.x, robot_pose.pose.orientation.y,
      robot_pose.pose.orientation.z, robot_pose.pose.orientation.w);
  std::vector<Point2> raw;
  raw.reserve(orig_global_plan.size());
  for (const auto& p : orig_global_plan)
  {
    raw.push_back({p.pose.position.x, p.pose.position.y});
  }
  {
    std::lock_guard<std::mutex> lock(params_mutex_);
    path_ref_.build(raw, !go_forward_, path_ds_, yaw_robot, kappa_half_span_);
    speed_limits_.v_max = max_vel_x_;
    speed_limits_reverse_.v_max = max_vel_x_;
    SpeedLimits limits = go_forward_ ? speed_limits_ : speed_limits_reverse_;
    const MpcParams& mp = mpc_.params();
    limits.wheelbase = mp.wheelbase;
    limits.steer_rate = mp.max_steer_rate;
    computeSpeedProfile(path_ref_, limits);
    mpc_.reset();

    // The robot cannot follow curves tighter than its steering limit allows
    const double kappa_max = std::tan(mp.max_steer) / mp.wheelbase;
    std::size_t i_tight = 0;
    for (std::size_t i = 1; i < path_ref_.size(); ++i)
    {
      if (fabs(path_ref_.at(i).kappa) > fabs(path_ref_.at(i_tight).kappa))
      {
        i_tight = i;
      }
    }
    const RefPoint& tight = path_ref_.at(i_tight);
    if (fabs(tight.kappa) > kappa_max)
    {
      ROS_WARN("Path curve radius %.2f m at (%.2f, %.2f) is tighter than the robot's minimum "
               "turning radius %.2f m (max_angular_z %.2f rad): the robot will cut or overshoot it",
               1.0 / fabs(tight.kappa), tight.x, tight.y, 1.0 / kappa_max, max_angular_z_);
    }
  }
  path_hint_ = -1;
  last_steer_cmd_ = steer_geom_.leftToCentre(steering_angle_);
  last_vel_cmd_ = 0.0;
  correction_attempts_ = 0;
  ROS_INFO("Local planner: path %.2f m, %zu samples, %s", path_ref_.length(),
           path_ref_.size(), go_forward_ ? "forward" : "reverse");

  // get waypoints low speed
  array_waypoints_low_speed_.Waypoints.clear();
  agv_msgs::ArrayWaypoints srv;
  client_get_waypoints_low_speed_.call(srv);
  array_waypoints_low_speed_ = srv.response;

  // get waypoints special safety
  array_waypoints_special_safety_.Waypoints.clear();
  agv_msgs::ArraySafetyWaypoints srv1;
  client_get_waypoints_special_safety_.call(srv1);
  array_waypoints_special_safety_ = srv1.response;

  // Reset state
  goal_reached_ = false;
  is_near_goal_ = false;
  rotate_to_goal_ = false;
  correcting_position_after_rotate_ = false;
  low_speed_ = false;
  steering_lagging_ = false;
  {
    std::lock_guard<std::mutex> lock(params_mutex_);
    setHeadingPriority(false);
    mpc_.reset();
  }

  safety_job_ = -1;
  // Zone speed limit is re-sent by set_safety while robot is inside a zone
  max_vel_requested = abs(max_vel_x_);
  robot_vel_.linear.x = 0.0;

  old_time_ = ros::Time::now().toSec();

  // Publish global path
  path_global_.header.stamp = ros::Time::now();
  path_global_.header.frame_id = "map";
  path_global_.poses = orig_global_plan;
  pub_path_.publish(path_global_);
  ROS_INFO("Local planner: set plan done");
  return true;
}

// Zone / direction safety fields are set by set_safety_job.py.
// Planner only overrides /safety_control near special safety points.
void MpcAgfPlannerROS::setSafety(double distance_to_goal)
{
  if (go_forward_)
  {
    switch (safety_job_) {
      case 4:
        msg_safety_.ints = {4, 8, 8, 11};
        break;
      default:
        return;
    }
  }
  else
  {
    switch (safety_job_) {
      // RANGE BASE SAFETY MODE: Adjust safety levels based on distance to goal
      case 3:
        if (abs(distance_to_goal) < 1.1)
          msg_safety_.ints = {5, 2, 2, 11};
        else if (abs(distance_to_goal) < 1.3)
          msg_safety_.ints = {5, 2, 2, 10};
        else if (abs(distance_to_goal) < 1.5)
          msg_safety_.ints = {5, 2, 2, 9};
        else
          msg_safety_.ints = {5, 2, 2, 8};
        break;
      //disable safety fork when near zone
      case 4:
        if (abs(distance_to_goal) < 1.1)
          msg_safety_.ints = {6, 8, 8, 6};
        else if (abs(distance_to_goal) < 1.3)
          msg_safety_.ints = {4, 4, 4, 6};
        else if (abs(distance_to_goal) < 1.5)
          msg_safety_.ints = {4, 4, 4, 9};
        else
          msg_safety_.ints = {4, 4, 4, 8};
        break;
      default:
        return;
    }
  }
  pub_safety_control_.publish(msg_safety_);
}

bool MpcAgfPlannerROS::checkRobotNearSlowPoint(
    const geometry_msgs::PoseStamped& pose_robot)
{
  if (array_waypoints_low_speed_.Waypoints.size() == 0)
  {
    return false;
  }

  for (auto i = 0; i < array_waypoints_low_speed_.Waypoints.size(); i++)
  {
    double dx = array_waypoints_low_speed_.Waypoints[i].pose.position.x -
                pose_robot.pose.position.x;
    double dy = array_waypoints_low_speed_.Waypoints[i].pose.position.y -
                pose_robot.pose.position.y;
    double distance = sqrt(dx * dx + dy * dy);
    if (distance < error_to_low_speed_points_)
    {
      return true;
    }
  }

  return false;
}

int MpcAgfPlannerROS::checkRobotNearSpecialSafety(
    const geometry_msgs::PoseStamped& pose_robot)
{
  if (array_waypoints_special_safety_.Waypoints.size() == 0)
  {
    return -1;
  }

  int size_wps = array_waypoints_special_safety_.Waypoints.size();
  for (auto i = 1; i <= size_wps; i++)
  {
    double dx = array_waypoints_special_safety_.Waypoints[size_wps - i].Pose.pose.position.x -
                pose_robot.pose.position.x;
    double dy = array_waypoints_special_safety_.Waypoints[size_wps - i].Pose.pose.position.y -
                pose_robot.pose.position.y;
    double distance = sqrt(dx * dx + dy * dy);
    if (distance < error_to_special_safety_points_)
    {
      return array_waypoints_special_safety_.Waypoints[size_wps - i].safety_job;
    }
  }

  return -1;
}

bool MpcAgfPlannerROS::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
{
  std::string dummy_message;
  geometry_msgs::PoseStamped dummy_pose;
  geometry_msgs::TwistStamped dummy_velocity, cmd_vel_stamped;
  uint32_t outcome = computeVelocityCommands(dummy_pose, dummy_velocity,
                                             cmd_vel_stamped, dummy_message);
  cmd_vel = cmd_vel_stamped.twist;
  // return true if outcome = 0
  return outcome == mbf_msgs::ExePathResult::SUCCESS;
}

uint32_t MpcAgfPlannerROS::computeVelocityCommands(
    const geometry_msgs::PoseStamped& pose,
    const geometry_msgs::TwistStamped& velocity,
    geometry_msgs::TwistStamped& cmd_vel, std::string& message)
{
  // Get dt
  new_time_ = ros::Time::now().toSec();
  dt_ = new_time_ - old_time_;
  old_time_ = new_time_;

  static uint32_t seq = 0;
  cmd_vel.header.seq = seq++;
  cmd_vel.header.stamp = ros::Time::now();
  cmd_vel.header.frame_id = robot_base_frame_;
  cmd_vel.twist.linear.x = cmd_vel.twist.linear.y = cmd_vel.twist.angular.z = 0;
  goal_reached_ = false;

  // Get robot pose
  geometry_msgs::PoseStamped robot_pose;
  costmap_ros_->getRobotPose(robot_pose);

  //check near pose slow speed
  low_speed_ = checkRobotNearSlowPoint(robot_pose);

  double error_goal_x =
      global_goal_.pose.position.x - robot_pose.pose.position.x;
  double error_goal_y =
      global_goal_.pose.position.y - robot_pose.pose.position.y;
  double distance_to_goal =
      sqrt(error_goal_x * error_goal_x + error_goal_y * error_goal_y);

  if (allow_set_safety_)
  {
    int current_safety_job = checkRobotNearSpecialSafety(robot_pose);
    if (current_safety_job != -1)
      safety_job_ = current_safety_job;
    std::cout << "error_to_special_safety_points: " << error_to_special_safety_points_ << std::endl;
    std::cout << "safety_job: " << safety_job_ << std::endl;
    std::cout << "distance_to_goal: " << distance_to_goal << std::endl;
    setSafety(distance_to_goal);
  }

  // Get velocity of robot
  ros::spinOnce();

  // Get yaw of robot
  double yaw_robot = quaternionToRPY(
      robot_pose.pose.orientation.x, robot_pose.pose.orientation.y,
      robot_pose.pose.orientation.z, robot_pose.pose.orientation.w);

  // Control robot
  // MPC model is referenced at base_link (fixed axle), no control point offset
  if (rotate_to_goal_ == false)
  {
    trackPath(robot_pose, yaw_robot, cmd_vel);
  }
  else
  {
    double yaw_goal = quaternionToRPY(
        global_goal_.pose.orientation.x, global_goal_.pose.orientation.y,
        global_goal_.pose.orientation.z, global_goal_.pose.orientation.w);
    double diff_angle = normalizeAngle(yaw_goal - yaw_robot);
    // ROS_INFO("diff_angle: %f", diff_angle);

    if (is_carlike_)
    {
      // Skip the steer-and-roll manoeuvre when the heading is already within
      // tolerance: rolling at full lock would only push the robot sideways
      bool rotated = fabs(diff_angle) < yaw_goal_tolerance_ ||
                     rotateCarLike(cmd_vel, robot_pose, diff_angle);
      if (rotated)
      {
        // Rotation completed, now check if position is still within tolerance.
        // Use the same reference point as the tracker's arrival check.
        double ref_x = robot_pose.pose.position.x + cos(yaw_robot) * ref_point_offset_;
        double ref_y = robot_pose.pose.position.y + sin(yaw_robot) * ref_point_offset_;
        double error_goal_x = global_goal_.pose.position.x - ref_x;
        double error_goal_y = global_goal_.pose.position.y - ref_y;
        double position_error = sqrt(error_goal_x * error_goal_x + error_goal_y * error_goal_y);

        if (position_error > xy_goal_tolerance_ &&
            correction_attempts_ < max_correction_attempts_)
        {
          // Position drifted during rotation, need to correct it
          ++correction_attempts_;
          ROS_WARN("Position error after rotation: %.3f m (tolerance: %.3f m). Correcting position (attempt %d/%d)...",
                   position_error, xy_goal_tolerance_, correction_attempts_,
                   max_correction_attempts_);

          // Reset flags to go back to position correction mode
          rotate_to_goal_ = false;
          is_near_goal_ = false;
          goal_reached_ = false;
          correcting_position_after_rotate_ = true;  // Mark that we're correcting position
          last_vel_cmd_ = 0.0;

          // Don't return SUCCESS yet, continue with position correction
          // trackPath() will be called next iteration to fix position
        }
        else
        {
          if (position_error > xy_goal_tolerance_)
          {
            ROS_WARN("Position error %.3f m still above tolerance after %d corrections, accepting goal",
                     position_error, correction_attempts_);
          }
          ROS_INFO("Goal reached! Position error: %.3f m, Angle error: %.3f rad",
                   position_error, diff_angle);
          goal_reached_ = true;
          correcting_position_after_rotate_ = false;
          return mbf_msgs::ExePathResult::SUCCESS;
        }
      }
    }
    else
    {
      if (fabs(diff_angle) < yaw_goal_tolerance_)
      {
        cmd_vel.twist.linear.x = 0;
        cmd_vel.twist.angular.z = 0;
        goal_reached_ = true;
        return mbf_msgs::ExePathResult::SUCCESS;
      }
      double angular_z = 0.8 * diff_angle;
      angular_z = std::min(angular_z, max_angular_z_);
      angular_z = std::max(angular_z, -max_angular_z_);
      cmd_vel.twist.linear.x = 0;
      cmd_vel.twist.angular.z = angular_z;
    }
  }
  // Nếu không return SUCCESS trong hàm này thì cả 2 planner bị gọi lại liên tục
  return mbf_msgs::ExePathResult::SUCCESS;
}

bool MpcAgfPlannerROS::rotateCarLike(
    geometry_msgs::TwistStamped& cmd_vel,
    const geometry_msgs::PoseStamped& robot_pose, double diff_angle)
{
  static int state = 0, prev_state = -1, dir = 0;
  static double target_steering = 0.0;
  static double start_diff_angle = 0.0;
  static double last_time = ros::Time::now().toSec();
  static double new_time = last_time;
  new_time = ros::Time::now().toSec();
  // Auto reset state after 1 second for next action
  if (new_time - last_time > 1.0)
  {
    state = 0;
    prev_state = -1;
  }
  last_time = new_time;

  if (state != prev_state)
  {
    if (state == 0)
    {
      ROS_INFO("---Begin rotate car like---");
    }
    else
    {
      ROS_INFO("Rotate carlike state: %d => %d", prev_state, state);
    }
    prev_state = state;
  }

  // Calculate steering and direction ONLY when starting rotation (state 0)
  // This prevents oscillation as diff_angle changes during rotation
  if (state == 0)
  {
    start_diff_angle = diff_angle;

    // Calculate position error: are we behind or past the goal?
    double dx = global_goal_.pose.position.x - robot_pose.pose.position.x;
    double dy = global_goal_.pose.position.y - robot_pose.pose.position.y;
    double yaw_robot = quaternionToRPY(
        robot_pose.pose.orientation.x, robot_pose.pose.orientation.y,
        robot_pose.pose.orientation.z, robot_pose.pose.orientation.w);

    // Project position error onto robot's forward direction
    double forward_error = dx * cos(yaw_robot) + dy * sin(yaw_robot);

    // If positive: goal is ahead (roll forward), if negative: goal is behind (roll backward)
    dir = (forward_error >= 0.0) ? 1 : -1;

    // Yaw rate has the sign of steer_sign * dir * steering (the MPC's model),
    // so the steering side depends on the roll direction as well as on the
    // turn: rolling backward with the same steering turns the other way
    double turn = (diff_angle >= 0.0) ? 1.0 : -1.0;  // +1 = turn left
    target_steering = (turn * dir * steer_geom_.steer_sign > 0.0)
                          ? steering_angle_for_rotate_max_
                          : steering_angle_for_rotate_min_;

    ROS_INFO("Rotate CarLike START: diff_angle=%.3f rad (%.1f deg), Turn %s, target_steering=%.3f rad (%.1f deg), forward_error=%.3f m, dir=%s",
                      diff_angle, diff_angle * 180.0 / M_PI,
                      (diff_angle >= 0.0) ? "LEFT" : "RIGHT",
                      target_steering, target_steering * 180.0 / M_PI,
                      forward_error, (dir > 0) ? "FORWARD" : "BACKWARD");
  }

  ROS_INFO_THROTTLE(0.5, "Rotate CarLike state=%d: diff_angle=%.3f rad (%.1f deg), using target_steering=%.3f rad, dir=%s",
                    state, diff_angle, diff_angle * 180.0 / M_PI,
                    target_steering, (dir > 0) ? "FORWARD" : "BACKWARD");

  switch (state)
  {
  case 0:  // ROTATE_STEERING
    cmd_vel.twist.linear.x = 0.0;
    cmd_vel.twist.angular.z = target_steering;
    if (abs(steering_angle_ - target_steering) <
        steering_angle_tolerance_)
    {
      state = 1;
    }
    break;
  case 1:  // DELAY_AF_ROTATE
    state = 2;
    break;
  case 2:  // ROLLING_MAIN_WHEEL
  {
    cmd_vel.twist.angular.z = target_steering;
    // Slow down over the last 0.3 rad so the stop lands near the target heading
    const double slow_angle = 0.3, v_roll_min = 0.05;
    double v_roll = max_trans_vel_for_rotate_ * std::min(1.0, fabs(diff_angle) / slow_angle);
    cmd_vel.twist.linear.x = std::max(v_roll_min, v_roll) * dir;

    // Also stop when the error changes sign: the wheel stays locked to one
    // side, so after an overshoot the robot would keep circling
    if (abs(diff_angle) < yaw_goal_tolerance_ ||
        diff_angle * start_diff_angle < 0.0)
    {
      cmd_vel.twist.linear.x = 0.0;
      state = 3;
    }
    break;
  }
  case 3:  // DELAY_AF_ROLLING
    state = 4;
    break;
  case 4:  // RETURN_STEERING
    cmd_vel.twist.angular.z = 0.0;
    if (abs(steering_angle_) < steering_angle_tolerance_)
    {
      state = 5;
    }
    break;
  case 5:  // DELAY_AF_STEERING
    state = 0;
    return true;
    break;

  default:
    break;
  }
  // ROS_INFO("linear.x: %f, angular.z: %f", cmd_vel.twist.linear.x,
  //          cmd_vel.twist.angular.z);
  return false;
}

/**
 * @brief Follows the global path with the LTV-MPC.
 *
 * - Reference point: fixed-axle midpoint (base_link + ref_point_offset_), where
 *   the kinematic model holds.
 * - Speed: path speed profile (curvature + braking to the goal), capped by
 *   max_vel_x, the speed-limit request and low-speed zones, ramped with a_acc.
 * - Steering: one MPC solve per cycle. Below v_min_mpc the last steering
 *   command is held, because steering cannot reduce the error at zero speed.
 * - Arrival: within xy_goal_tolerance of the goal, or past the goal at the end
 *   of the path. Then onPositionReached() starts the final rotation.
 */
void MpcAgfPlannerROS::trackPath(const geometry_msgs::PoseStamped& robot_pose,
                                   double yaw_robot,
                                   geometry_msgs::TwistStamped& cmd_vel)
{
  std::lock_guard<std::mutex> lock(params_mutex_);
  cmd_vel.twist.linear.x = 0.0;
  cmd_vel.twist.angular.z = 0.0;
  if (path_ref_.empty())
  {
    return;
  }
  const double dir = go_forward_ ? 1.0 : -1.0;
  const double dt = std::max(0.0, std::min(dt_, 0.2));

  // Reference point and path errors
  double ref_x = robot_pose.pose.position.x + cos(yaw_robot) * ref_point_offset_;
  double ref_y = robot_pose.pose.position.y + sin(yaw_robot) * ref_point_offset_;
  std::size_t window = static_cast<std::size_t>(2.0 / path_ds_) + 1;
  std::size_t idx = path_ref_.nearest(ref_x, ref_y, path_hint_, window);
  path_hint_ = static_cast<long>(idx);
  PathErrors err = path_ref_.errors(ref_x, ref_y, yaw_robot, idx);

  // Arrival check
  const RefPoint& goal = path_ref_.back();
  double gx = goal.x - ref_x;
  double gy = goal.y - ref_y;
  double position_error = sqrt(gx * gx + gy * gy);
  // Distance left along the goal heading; negative = past the goal
  double to_goal_along = (cos(goal.yaw) * gx + sin(goal.yaw) * gy) * dir;
  // Send the stop early by the distance the robot coasts after it, so it
  // comes to rest at the goal instead of past it
  // With brake parameters set, the distance comes from the measured speed instead
  const bool use_brake = brake_delay_ > 0.0 || brake_decel_ > 0.0;
  double coast = fabs(last_vel_cmd_) * (go_forward_ ? stop_lead_time_ : stop_lead_time_reverse_);
  if (use_brake)
  {
    double speed_now = fabs(measuredSpeed());
    coast = creepDistance(speed_now) + 0.5 * speed_now * dt;
  }
  double stop_along = to_goal_along - coast;
  double lateral = -sin(goal.yaw) * gx + cos(goal.yaw) * gy;
  double stop_error = sqrt(stop_along * stop_along + lateral * lateral);
  bool at_path_end = err.s >= path_ref_.length() - path_ds_ - coast;
  // The brake model says exactly where to stop, the tolerance would stop it early
  if ((!use_brake && stop_error < xy_goal_tolerance_) || (at_path_end && stop_along < 0.0))
  {
    if (use_brake && coast > to_goal_along + xy_goal_tolerance_)
    {
      ROS_WARN("Creep %.3f m exceeds remaining %.3f m + tolerance, robot will overshoot the goal",
               coast, to_goal_along);
    }
    ROS_INFO("Arrived: error %.3f m (along %.3f m, lateral %.3f m), coast %.3f m, cmd %.3f m/s, measured %.3f m/s",
             position_error, to_goal_along, err.e_y, coast, last_vel_cmd_, robot_vel_.linear.x);
    last_vel_cmd_ = 0.0;
    steering_lagging_ = false;
    onPositionReached(position_error);
    return;
  }
  if (correcting_position_after_rotate_)
  {
    ROS_INFO_THROTTLE(1.0, "Correcting position: %.3f m remaining (tolerance: %.3f m)",
                      position_error, xy_goal_tolerance_);
  }

  // Speed: profile and limits, ramped from the measured speed
  auto speed_limit = [this](double v_profile) {
    double v = std::min(v_profile, max_vel_x_);
    v = std::min(v, max_vel_requested);
    if (low_speed_ || steering_lagging_)
    {
      v = std::min(v, vel_x_when_more_angle_);
    }
    return std::max(0.0, v);
  };
  double v_target = speed_limit(path_ref_.sampleAtS(err.s).v);
  // Ramp from the last command, not the measured speed: the drive lags its
  // command, so ramping from the measurement only ever asks for a small step
  // below the current speed and the robot brakes far softer than a_brake.
  // Speeding up stays within a margin of the measured speed so the command
  // does not wind up while the robot is held back (e.g. by the safety system).
  const double windup_margin = 0.15;  // m/s
  double v_now = fabs(robot_vel_.linear.x);
  double v_last = fabs(last_vel_cmd_);
  double v_cmd = v_target > v_last
                     ? std::min(v_target, std::min(v_last, v_now + windup_margin) + a_acc_ * dt)
                     : std::max(v_target, v_last - a_brake_ * dt);

  // Heading first when backing up close to the goal
  setHeadingPriority(!go_forward_ && priority_angle_adjustment_ && position_error < 0.65);

  // Horizon: speed and curvature at each predicted step
  const MpcParams& mp = mpc_.params();
  const int N = mp.horizon;
  std::vector<double> v_h(N), k_h(N + 1), s_h(N + 1);
  double s = err.s;
  for (int k = 0; k <= N; ++k)
  {
    RefPoint rp = path_ref_.sampleAtS(s);
    s_h[k] = s;
    k_h[k] = rp.kappa;
    if (k < N)
    {
      double vk = std::min(speed_limit(rp.v), v_cmd + a_acc_ * mp.dt * k);
      v_h[k] = dir * vk;
      s += vk * mp.dt;
    }
  }

  // Steering
  // MPC state and commands are centre angles; the wheel reports its own angle
  const double delta_centre = steer_geom_.leftToCentre(steering_angle_);
  MpcState x0{err.e_y, err.e_psi, delta_centre};
  MpcResult res;
  if (v_cmd < v_min_mpc_)
  {
    res.delta_cmd = last_steer_cmd_;
  }
  else
  {
    x0 = mpc_.compensateDelay(x0, dir * v_cmd, k_h[0], last_steer_cmd_);
    res = mpc_.solve(x0, v_h, k_h, last_steer_cmd_);
    if (!res.solved)
    {
      ROS_WARN_THROTTLE(1.0, "MPC solve failed, using fallback steering");
    }
  }

  last_vel_cmd_ = dir * v_cmd;
  last_steer_cmd_ = res.delta_cmd;
  cmd_vel.twist.linear.x = last_vel_cmd_;
  double wheel_cmd = steer_geom_.centreToLeft(res.delta_cmd);
  cmd_vel.twist.angular.z = std::max(min_steering_angle_, std::min(max_steering_angle_, wheel_cmd));

  // Required steering rate scales with speed, so slow down while the wheel
  // cannot follow the command; applied from the next cycle
  steering_lagging_ = steering_lag_slowdown_ > 0.0 &&
                      fabs(cmd_vel.twist.angular.z - steering_angle_) > steering_lag_slowdown_;
  if (steering_lagging_)
  {
    ROS_WARN_THROTTLE(1.0, "Steering lagging: requested %.3f, measured %.3f rad, slowing down",
                      cmd_vel.twist.angular.z, steering_angle_);
  }

  publishTrackingDebug(res, err, last_vel_cmd_, s_h);
}

void MpcAgfPlannerROS::setHeadingPriority(bool active)
{
  // Must be called with params_mutex_ held. setParams() drops the warm start,
  // so only touch the weight when the state changes.
  if (active == heading_priority_active_)
  {
    return;
  }
  heading_priority_active_ = active;
  MpcParams p = mpc_.params();
  p.q_epsi = active ? 8.0 * base_q_epsi_ : base_q_epsi_;
  mpc_.setParams(p);
}

double MpcAgfPlannerROS::measuredSpeed()
{
  nav_msgs::Odometry odom;
  odom_helper_.getOdom(odom);
  if (!odom.header.stamp.isZero() &&
      (ros::Time::now() - odom.header.stamp).toSec() < odom_timeout_)
  {
    return odom.twist.twist.linear.x;
  }
  ROS_WARN_THROTTLE(5.0, "Odometry is old or missing, creep uses /cmd_vel_feedback speed");
  return robot_vel_.linear.x;
}

double MpcAgfPlannerROS::creepDistance(double speed) const
{
  // Keeps the speed until the brake acts, then v^2 / (2a) to stand still
  double v = fabs(speed);
  double distance = v * brake_delay_;
  if (brake_decel_ > 0.0)
  {
    distance += v * v / (2.0 * brake_decel_);
  }
  return distance;
}

void MpcAgfPlannerROS::onPositionReached(double position_error)
{
  // Reset near-goal flag for next path
  is_near_goal_ = false;

  // Check if we just corrected position after rotation
  if (correcting_position_after_rotate_)
  {
    // Position corrected, now retry rotation
    ROS_INFO("Position correction complete (%.3f m). Retrying final rotation...", position_error);
    correcting_position_after_rotate_ = false;
    rotate_to_goal_ = true;  // Trigger rotation again
  }
  // Determine if final rotation to goal orientation is needed
  else if (ignore_last_rotate_)
  {
    // Skip final rotation, mark goal as reached immediately
    goal_reached_ = true;
  }
  else
  {
    // Trigger final rotation to match goal orientation
    rotate_to_goal_ = true;
  }
}

void MpcAgfPlannerROS::publishTrackingDebug(const MpcResult& mpc,
                                              const PathErrors& err,
                                              double v_cmd,
                                              const std::vector<double>& s_pred)
{
  ros::Time now = ros::Time::now();
  auto toPose = [&](double x, double y, double yaw) {
    geometry_msgs::PoseStamped p;
    p.header.stamp = now;
    p.header.frame_id = global_frame_;
    p.pose.position.x = x;
    p.pose.position.y = y;
    p.pose.orientation.z = sin(yaw / 2.0);
    p.pose.orientation.w = cos(yaw / 2.0);
    return p;
  };

  // Remaining path, at most 10 m ahead
  nav_msgs::Path local_path_msg;
  local_path_msg.header.stamp = now;
  local_path_msg.header.frame_id = global_frame_;
  for (std::size_t i = static_cast<std::size_t>(std::max(0L, path_hint_));
       i < path_ref_.size() && path_ref_.at(i).s <= err.s + 10.0; ++i)
  {
    const RefPoint& rp = path_ref_.at(i);
    local_path_msg.poses.push_back(toPose(rp.x, rp.y, rp.yaw));
  }
  pub_local_path_.publish(local_path_msg);

  // MPC prediction mapped back to the map frame
  nav_msgs::Path pred;
  pred.header.stamp = now;
  pred.header.frame_id = global_frame_;
  for (std::size_t k = 0; k < mpc.e_y_pred.size() && k + 1 < s_pred.size(); ++k)
  {
    RefPoint rp = path_ref_.sampleAtS(s_pred[k + 1]);
    double e_y = mpc.e_y_pred[k];
    pred.poses.push_back(toPose(rp.x - sin(rp.yaw) * e_y, rp.y + cos(rp.yaw) * e_y, rp.yaw));
  }
  pub_mpc_predicted_.publish(pred);

  // Reference point (blue sphere)
  RefPoint ref = path_ref_.sampleAtS(err.s);
  visualization_msgs::Marker marker;
  marker.header.stamp = now;
  marker.header.frame_id = global_frame_;
  marker.ns = "reference_point";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose = toPose(ref.x, ref.y, ref.yaw).pose;
  marker.scale.x = 0.15;
  marker.scale.y = 0.15;
  marker.scale.z = 0.15;
  marker.color.b = 1.0;
  marker.color.a = 1.0;
  pub_reference_point_.publish(marker);

  // [e_y, e_psi, delta_centre, delta_cmd_centre, v, solve_ms]
  std_msgs::Float64MultiArray e;
  // Steering values are centre angles (model space), not the left wheel's
  e.data = {err.e_y, err.e_psi, steer_geom_.leftToCentre(steering_angle_),
            mpc.delta_cmd, v_cmd, mpc.solve_ms};
  pub_tracking_error_.publish(e);
}

double MpcAgfPlannerROS::quaternionToRPY(double qx, double qy, double qz,
                                           double qw)
{
  // The incoming geometry_msgs::Quaternion is transformed to a tf::Quaterion
  tf2::Quaternion quat;
  quat.setX(qx);
  quat.setY(qy);
  quat.setZ(qz);
  quat.setW(qw);

  // The tf::Quaternion has a method to acess roll pitch and yaw
  double roll, pitch, yaw;
  tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
  return yaw;
}

double MpcAgfPlannerROS::normalizeAngle(double angle)
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

bool MpcAgfPlannerROS::isGoalReached()
{
  if (!initialized_)
  {
    ROS_ERROR("This planner has not been initialized, please call initialize() "
              "before using this planner");
    return mbf_msgs::ExePathResult::NOT_INITIALIZED;
  }

  if (goal_reached_)
  {
    ROS_INFO("GOAL Reached!");
    return true;
  }
  return false;
}

}  // namespace mpc_agf_planner
