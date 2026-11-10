#ifndef MPC_AGF_PLANNER_H_
#define MPC_AGF_PLANNER_H_

#include <ros/ros.h>

#include <nlohmann/json.hpp>

// base local planner base class and utilities
#include <base_local_planner/costmap_model.h>
#include <base_local_planner/goal_functions.h>
#include <base_local_planner/odometry_helper_ros.h>
#include <mbf_costmap_core/costmap_controller.h>
#include <nav_core/base_local_planner.h>

// message types
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <visualization_msgs/Marker.h>
#include <agv_msgs/ArrayWaypoints.h>
#include <agv_msgs/ArrayWaypointsResponse.h>
#include <agv_msgs/WaypointSafety.h>
#include <agv_msgs/ArraySafetyWaypoints.h>
#include <agf_msgs/Int8Array.h>
#include <std_stamped_msgs/StringStamped.h>

// transforms
#include <tf2/utils.h>
#include <tf2_ros/buffer.h>

// dynamic reconfigure
#include <mpc_agf_planner/MpcAgfPlannerReconfigureConfig.h>
#include <dynamic_reconfigure/server.h>

// costmap
#include <costmap_2d/costmap_2d_ros.h>

// path tracking
#include <mpc_agf_planner/mpc_controller.h>
#include <mpc_agf_planner/path_reference.h>
#include <mpc_agf_planner/speed_profile.h>
#include <mpc_agf_planner/steering_geometry.h>

#include <mutex>

namespace mpc_agf_planner
{
class MpcAgfPlannerROS : public nav_core::BaseLocalPlanner,
                         public mbf_costmap_core::CostmapController
{
public:
  MpcAgfPlannerROS();

  ~MpcAgfPlannerROS();

  void initialize(std::string name, tf2_ros::Buffer* tf,
                  costmap_2d::Costmap2DROS* costmap_ros);

  bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);

  void setSafety(double distance_to_goal);

  bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);

  uint32_t computeVelocityCommands(const geometry_msgs::PoseStamped& pose,
                                   const geometry_msgs::TwistStamped& velocity,
                                   geometry_msgs::TwistStamped& cmd_vel,
                                   std::string& message);

  /**
   * Follow the path with the MPC: speed from the speed profile, steering from
   * one QP solve. Also detects arrival and starts the final rotation.
   */
  void trackPath(const geometry_msgs::PoseStamped& robot_pose, double yaw_robot,
                 geometry_msgs::TwistStamped& cmd_vel);

  /** Current speed of base_link: odometry if fresh, else /cmd_vel_feedback */
  double measuredSpeed();

  /** Distance travelled after the stop command: brake delay, then braking */
  double creepDistance(double speed) const;

  bool rotateCarLike(geometry_msgs::TwistStamped& cmd_vel,
                     const geometry_msgs::PoseStamped& robot_pose,
                     double diff_angle);

  bool checkRobotNearSlowPoint(const geometry_msgs::PoseStamped& pose_robot);

  int checkRobotNearSpecialSafety(const geometry_msgs::PoseStamped& pose_robot);

  double quaternionToRPY(double qx, double qy, double qz, double qw);

  double normalizeAngle(double angle);

  void cmdVelCallback(const geometry_msgs::Twist::ConstPtr msg);

  // void safetyCallback(const safety_msgs::SafetyStatus::ConstPtr msg);

  void request_speed_limit_cb(std_stamped_msgs::StringStamped::ConstPtr msg);


  void setRuleSafety();

  void setDirection(std::string direction);

  void steeringAngleCallback(const std_msgs::Float64::ConstPtr msg);

  void reconfigureCB(MpcAgfPlannerReconfigureConfig& config, uint32_t level);

  bool isGoalReached();

  bool isGoalReached(double xy_tolerance, double yaw_tolerance)
  {
    return isGoalReached();
  };

  bool cancel() { return false; };

private:
  // Called on arrival at the goal position (also after a position correction)
  void onPositionReached(double position_error);
  void setHeadingPriority(bool active);
  void publishTrackingDebug(const MpcResult& mpc, const PathErrors& err,
                            double v_cmd, const std::vector<double>& s_pred);
  MpcParams loadMpcParams(ros::NodeHandle& nh) const;

  costmap_2d::Costmap2DROS* costmap_ros_;
  costmap_2d::Costmap2D* costmap_;
  tf2_ros::Buffer* tf_;

  boost::shared_ptr<base_local_planner::CostmapModel> costmap_model_;
  boost::shared_ptr<dynamic_reconfigure::Server<MpcAgfPlannerReconfigureConfig>> dynamic_recfg_;

  std::string name_;
  std::string odom_topic_;
  std::string global_frame_;
  std::string robot_base_frame_;

  geometry_msgs::PoseStamped global_goal_;
  nav_msgs::Path path_global_;
  base_local_planner::OdometryHelperRos
      odom_helper_;  // Provides an interface to receive the current velocity from the robot
  geometry_msgs::Twist
      robot_vel_;  // Store current robot translational and angular velocity (vx, vy, omega)
  agf_msgs::Int8Array msg_safety_;

  ros::NodeHandle nh_;
  ros::Publisher pub_path_;
  ros::Publisher pub_safety_control_;
  
  // Visualization publishers
  ros::Publisher pub_local_path_;        // Remaining resampled path
  ros::Publisher pub_mpc_predicted_;     // MPC predicted trajectory
  ros::Publisher pub_reference_point_;   // Reference point on the path
  ros::Publisher pub_tracking_error_;    // [e_y, e_psi, delta_centre, delta_cmd_centre, v, solve_ms]

  ros::Subscriber sub_cmd_vel_;
  ros::Subscriber sub_angle_steer_;
  ros::Subscriber sub_safety_zone_;
  ros::Subscriber request_speed_limit_sub;

  ros::ServiceClient client_get_waypoints_low_speed_;
  ros::ServiceClient client_get_waypoints_special_safety_;


  bool initialized_;
  bool goal_reached_;
  bool is_near_goal_;
  bool go_forward_;
  bool rotate_to_goal_;
  bool correcting_position_after_rotate_;  // True when fixing position drift after rotation
  bool low_speed_;
  bool is_carlike_;
  bool ignore_last_rotate_;
  bool allow_set_safety_;

  int safety_job_;
  int correction_attempts_;            // Position corrections after final rotation
  int max_correction_attempts_;

  double max_vel_x_;                   // Maximun of linear velocity
  double max_angular_z_;               // Maximun of angle velocity
  double angle_to_global_when_start_;  // Angle to rotate to global path
  double error_to_start_stop_;         // Circle around goal robot begin stop
  double error_angle_stop_;  // Error angle allow between robot and goal
  double
      max_error_pose_to_path_;  // Check robot far the local path if > max_error_pose_to_local_ ==> reset global planner
  double old_time_;                // Use to know a loop time of control
  double new_time_;                // Use to know a loop time of control
  double dt_;                      // Loop time of control
  double steering_angle_;
  double steering_lag_slowdown_;  // rad, steering lag that triggers slow speed (0 = off)
  double brake_delay_;    // s, from stop command until the physical brake acts (0 = off)
  double brake_decel_;    // m/s^2, deceleration once the brake acts (0 = ignore braking distance)
  double odom_timeout_;   // s, odometry older than this is not used for speed
  bool steering_lagging_;  // measured steering lags the command
  bool priority_angle_adjustment_;  // heading first when backing up close to goal
  bool heading_priority_active_;    // q_epsi is scaled up
  double base_q_epsi_;              // q_epsi without the scaling
  double min_steering_angle_;   // rad, left wheel limit (turning right)
  double max_steering_angle_;   // rad, left wheel limit (turning left)
  double vel_x_when_more_angle_;

  // MPC path tracking
  PathReference path_ref_;
  MpcController mpc_;               // Works in centre-angle space
  SteeringGeometry steer_geom_;     // Left-wheel <-> centre angle (Ackermann)
  SpeedLimits speed_limits_;          // Forward driving
  SpeedLimits speed_limits_reverse_;  // Reverse: own braking/creep, the robot coasts further
  std::mutex params_mutex_;       // Guards MPC params against dynamic reconfigure
  long path_hint_;                 // Last reference index, -1 = search whole path
  double last_steer_cmd_;          // rad
  double last_vel_cmd_;            // m/s, signed
  double ref_point_offset_;        // base_link to fixed-axle midpoint (m)
  double a_acc_;                   // m/s^2
  double a_brake_;                 // Deceleration allowed when a limit drops (m/s^2)
  double v_min_mpc_;               // Below this speed steering is held (m/s)
  double stop_lead_time_;          // Stop is sent this long before reaching the goal (s)
  double stop_lead_time_reverse_;  // Same, when reversing (s)
  // Reverse stop tuning as configured; < 0 = use the forward value
  double a_dec_reverse_cfg_;
  double v_creep_reverse_cfg_;
  double creep_dist_reverse_cfg_;
  double stop_lead_time_reverse_cfg_;
  void updateReverseLimits();
  double path_ds_;                 // Path resampling step (m)
  double kappa_half_span_;         // Curvature smoothing, +- arc length (m)
  double steering_angle_for_rotate_;  // Góc bánh lái khi rotate (deprecated, use min/max)
  double steering_angle_for_rotate_min_;  // Min steering angle (right turn)
  double steering_angle_for_rotate_max_;  // Max steering angle (left turn)
  double steering_angle_tolerance_;   // Tolerance của góc lái khi rotate
  double max_trans_vel_for_rotate_;  // Vận tốc bánh transaction khi rotate
  double error_to_low_speed_points_;
  double error_to_special_safety_points_;

  double yaw_goal_tolerance_;
  double xy_goal_tolerance_;
  double init_max_vel_x_;
  double init_vel_x_when_more_angle_;
  double max_vel_requested;
  double init_vel_reduce_after_stop_;

  agv_msgs::ArrayWaypointsResponse array_waypoints_low_speed_;
  agv_msgs::ArraySafetyWaypointsResponse array_waypoints_special_safety_;
};
};  // namespace mpc_agf_planner
#endif