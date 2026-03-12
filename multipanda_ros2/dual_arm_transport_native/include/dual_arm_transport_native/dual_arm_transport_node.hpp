#ifndef DUAL_ARM_TRANSPORT_NATIVE__DUAL_ARM_TRANSPORT_NODE_HPP_
#define DUAL_ARM_TRANSPORT_NATIVE__DUAL_ARM_TRANSPORT_NODE_HPP_

#include <atomic>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "builtin_interfaces/msg/duration.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "controller_manager_msgs/srv/list_controllers.hpp"
#include "controller_manager_msgs/srv/switch_controller.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "moveit_msgs/srv/get_cartesian_path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "dual_arm_transport_native/cascade_pid.hpp"

namespace dual_arm_transport_native
{

class DualArmTransportNativeNode : public rclcpp::Node
{
public:
  explicit DualArmTransportNativeNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  using Fjt = control_msgs::action::FollowJointTrajectory;
  using GoalHandleFjt = rclcpp_action::ClientGoalHandle<Fjt>;

  void declare_and_load_parameters();
  std::vector<double> expand_joint_vector(const std::vector<double> & raw, double fallback) const;

  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void merged_trajectory_callback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg);
  void cartesian_delta_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
  void cartesian_targets_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
  void state6_task_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);

  bool normalize_trajectory(
    const trajectory_msgs::msg::JointTrajectory & in,
    trajectory_msgs::msg::JointTrajectory & out) const;
  bool derive_reference_velocity(
    const trajectory_msgs::msg::JointTrajectory & traj,
    std::vector<std::vector<double>> & ref_dq) const;
  bool sample_reference(
    const std::vector<double> & t_vec,
    const std::vector<std::vector<double>> & q_mat,
    const std::vector<std::vector<double>> & dq_mat,
    double t_query,
    std::vector<double> & q_ref,
    std::vector<double> & dq_ref) const;
  bool read_joint_feedback(std::vector<double> & q, std::vector<double> & dq) const;

  void execute_trajectory_task(const trajectory_msgs::msg::JointTrajectory & traj);
  void execute_cartesian_delta_task(
    double dx, double dy, double dz,
    double max_step, double target_speed, double time_scale);
  void execute_cartesian_targets_task(
    const geometry_msgs::msg::Pose & left_goal,
    const geometry_msgs::msg::Pose & right_goal,
    double max_step, double target_speed, double time_scale,
    bool avoid_collisions);
  void execute_state6_task(
    int mode,
    const std::vector<double> & payload);
  bool execute_merged_trajectory_internal(const trajectory_msgs::msg::JointTrajectory & traj);
  bool execute_with_cascade_pid(const trajectory_msgs::msg::JointTrajectory & traj);
  bool execute_with_action_fallback(const trajectory_msgs::msg::JointTrajectory & traj);
  bool execute_cartesian_delta_internal(
    double dx, double dy, double dz,
    double max_step, double target_speed, double time_scale);
  bool execute_cartesian_targets_internal(
    const geometry_msgs::msg::Pose & left_goal,
    const geometry_msgs::msg::Pose & right_goal,
    double max_step, double target_speed, double time_scale,
    bool avoid_collisions = false,
    double fraction_threshold_override = -1.0);
  bool execute_state6_translate_internal(const std::vector<double> & payload);
  bool execute_state6_rotate_internal(const std::vector<double> & payload);

  bool lookup_current_pose(const std::string & link_name, geometry_msgs::msg::Pose & pose_out) const;
  bool build_cartesian_traj_for_group(
    const std::string & group_name,
    const std::string & link_name,
    const geometry_msgs::msg::Pose & start_pose,
    const geometry_msgs::msg::Pose & goal_pose,
    double max_step,
    bool avoid_collisions,
    trajectory_msgs::msg::JointTrajectory & traj_out,
    double & fraction_out);
  bool build_merged_trajectory_from_lr(
    const trajectory_msgs::msg::JointTrajectory & left_traj,
    const trajectory_msgs::msg::JointTrajectory & right_traj,
    double time_scale,
    trajectory_msgs::msg::JointTrajectory & merged_out) const;

  std::unordered_map<std::string, std::string> list_controllers();
  bool switch_controllers(
    const std::vector<std::string> & activate,
    const std::vector<std::string> & deactivate);
  bool switch_to_velocity_controller();
  bool switch_to_trajectory_controller();

  void publish_velocity_command(const std::vector<double> & cmd);
  void publish_status(const std::string & text) const;

  static double duration_to_sec(const builtin_interfaces::msg::Duration & d);
  static builtin_interfaces::msg::Duration sec_to_duration(double sec);
  static double clamp(double v, double lo, double hi);

private:
  // ROS interfaces
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr merged_traj_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr cartesian_delta_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr cartesian_targets_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr state6_task_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joint_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;

  rclcpp::Client<controller_manager_msgs::srv::ListControllers>::SharedPtr list_ctrl_client_;
  rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr switch_ctrl_client_;
  rclcpp::Client<moveit_msgs::srv::GetCartesianPath>::SharedPtr cartesian_client_;
  rclcpp_action::Client<Fjt>::SharedPtr traj_action_client_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // cached joint state
  mutable std::mutex state_mutex_;
  std::vector<double> current_q_;
  std::vector<double> current_dq_;
  std::vector<bool> joint_seen_;

  // guard for single execution
  std::atomic<bool> executing_;

  // params
  std::vector<std::string> joint_names_;
  std::unordered_map<std::string, size_t> joint_index_;
  std::string merged_trajectory_topic_;
  std::string velocity_command_topic_;
  std::string status_topic_;
  std::string cartesian_delta_topic_;
  std::string cartesian_targets_topic_;
  std::string state6_task_topic_;
  std::string trajectory_action_name_;
  std::string trajectory_controller_name_;
  std::string velocity_controller_name_;
  bool use_cascade_joint_pid_;
  bool cascade_pid_auto_switch_controllers_;
  bool enable_native_cartesian_delta_;
  bool enable_native_state6_task_;
  std::string planning_frame_;
  std::string left_group_name_;
  std::string right_group_name_;
  std::string left_ee_link_;
  std::string right_ee_link_;
  double cartesian_fraction_threshold_;
  double cartesian_default_max_step_;
  double cartesian_default_target_speed_;
  double merge_default_time_scale_;
  double merge_min_dt_;
  double merge_max_joint_vel_;
  double merge_max_joint_acc_;
  double merge_vel_lpf_;

  double cascade_pid_loop_hz_;
  double cascade_pid_outer_i_clamp_;
  double cascade_pid_inner_i_clamp_;
  double cascade_pid_outer_v_limit_;
  double cascade_pid_cmd_max_vel_;
  double cascade_pid_cmd_max_acc_;
  double cascade_pid_finish_pos_tol_;
  double cascade_pid_finish_vel_tol_;
  double cascade_pid_finish_hold_sec_;
  double cascade_pid_timeout_pad_sec_;
  double action_goal_time_tolerance_sec_;

  std::vector<double> outer_kp_;
  std::vector<double> outer_ki_;
  std::vector<double> outer_kd_;
  std::vector<double> inner_kp_;
  std::vector<double> inner_ki_;
  std::vector<double> inner_kd_;
};

}  // namespace dual_arm_transport_native

#endif  // DUAL_ARM_TRANSPORT_NATIVE__DUAL_ARM_TRANSPORT_NODE_HPP_
