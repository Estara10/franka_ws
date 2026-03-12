#include "dual_arm_transport_native/dual_arm_transport_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <future>
#include <set>
#include <thread>

#include "tf2/exceptions.h"
#include "tf2/time.h"

namespace dual_arm_transport_native
{

using namespace std::chrono_literals;

namespace
{

geometry_msgs::msg::Quaternion rotate_quaternion_about_z(
  const geometry_msgs::msg::Quaternion & in_q,
  double yaw_rad)
{
  const double half = 0.5 * yaw_rad;
  const double sz = std::sin(half);
  const double cz = std::cos(half);

  // q_out = q_yaw * q_in
  geometry_msgs::msg::Quaternion out_q;
  out_q.x = (cz * in_q.x) - (sz * in_q.y);
  out_q.y = (cz * in_q.y) + (sz * in_q.x);
  out_q.z = (cz * in_q.z) + (sz * in_q.w);
  out_q.w = (cz * in_q.w) - (sz * in_q.z);

  const double n = std::sqrt(
    out_q.x * out_q.x + out_q.y * out_q.y + out_q.z * out_q.z + out_q.w * out_q.w);
  if (n > 1e-9) {
    out_q.x /= n;
    out_q.y /= n;
    out_q.z /= n;
    out_q.w /= n;
  } else {
    out_q = in_q;
  }
  return out_q;
}

}  // namespace

DualArmTransportNativeNode::DualArmTransportNativeNode(const rclcpp::NodeOptions & options)
: Node("dual_arm_transport_native_node", options),
  executing_(false)
{
  declare_and_load_parameters();

  joint_index_.clear();
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    joint_index_[joint_names_[i]] = i;
  }

  current_q_.assign(joint_names_.size(), 0.0);
  current_dq_.assign(joint_names_.size(), 0.0);
  joint_seen_.assign(joint_names_.size(), false);

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  joint_vel_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(velocity_command_topic_, 10);
  status_pub_ = create_publisher<std_msgs::msg::String>(status_topic_, 10);

  joint_states_sub_ = create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states", 50,
    std::bind(&DualArmTransportNativeNode::joint_state_callback, this, std::placeholders::_1));

  merged_traj_sub_ = create_subscription<trajectory_msgs::msg::JointTrajectory>(
    merged_trajectory_topic_, 10,
    std::bind(&DualArmTransportNativeNode::merged_trajectory_callback, this, std::placeholders::_1));

  cartesian_delta_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
    cartesian_delta_topic_, 10,
    std::bind(&DualArmTransportNativeNode::cartesian_delta_callback, this, std::placeholders::_1));
  cartesian_targets_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
    cartesian_targets_topic_, 10,
    std::bind(&DualArmTransportNativeNode::cartesian_targets_callback, this, std::placeholders::_1));
  state6_task_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
    state6_task_topic_, 10,
    std::bind(&DualArmTransportNativeNode::state6_task_callback, this, std::placeholders::_1));

  list_ctrl_client_ = create_client<controller_manager_msgs::srv::ListControllers>(
    "/controller_manager/list_controllers");
  switch_ctrl_client_ = create_client<controller_manager_msgs::srv::SwitchController>(
    "/controller_manager/switch_controller");
  cartesian_client_ = create_client<moveit_msgs::srv::GetCartesianPath>(
    "/compute_cartesian_path");

  traj_action_client_ = rclcpp_action::create_client<Fjt>(this, trajectory_action_name_);

  RCLCPP_INFO(
    get_logger(),
    "dual_arm_transport_native_node started. joints=%zu, merged_topic=%s, delta_topic=%s, "
    "targets_topic=%s, state6_topic=%s, mode=%s",
    joint_names_.size(), merged_trajectory_topic_.c_str(), cartesian_delta_topic_.c_str(),
    cartesian_targets_topic_.c_str(), state6_task_topic_.c_str(),
    use_cascade_joint_pid_ ? "cascade_pid" : "action_only");
}

void DualArmTransportNativeNode::declare_and_load_parameters()
{
  const std::vector<std::string> default_joint_names = {
    "mj_left_joint1", "mj_left_joint2", "mj_left_joint3", "mj_left_joint4",
    "mj_left_joint5", "mj_left_joint6", "mj_left_joint7",
    "mj_right_joint1", "mj_right_joint2", "mj_right_joint3", "mj_right_joint4",
    "mj_right_joint5", "mj_right_joint6", "mj_right_joint7"
  };

  const std::vector<double> default_outer_kp = {
    4.2, 4.2, 4.2, 4.0, 3.2, 3.0, 2.6,
    4.2, 4.2, 4.2, 4.0, 3.2, 3.0, 2.6
  };
  const std::vector<double> default_outer_ki = {
    0.06, 0.06, 0.06, 0.05, 0.04, 0.04, 0.03,
    0.06, 0.06, 0.06, 0.05, 0.04, 0.04, 0.03
  };
  const std::vector<double> default_outer_kd = {
    0.11, 0.11, 0.11, 0.09, 0.07, 0.07, 0.05,
    0.11, 0.11, 0.11, 0.09, 0.07, 0.07, 0.05
  };
  const std::vector<double> default_inner_kp = {
    0.85, 0.85, 0.85, 0.80, 0.65, 0.60, 0.50,
    0.85, 0.85, 0.85, 0.80, 0.65, 0.60, 0.50
  };
  const std::vector<double> default_inner_ki = {
    0.12, 0.12, 0.12, 0.10, 0.08, 0.08, 0.06,
    0.12, 0.12, 0.12, 0.10, 0.08, 0.08, 0.06
  };
  const std::vector<double> default_inner_kd = {
    0.020, 0.020, 0.020, 0.018, 0.015, 0.015, 0.012,
    0.020, 0.020, 0.020, 0.018, 0.015, 0.015, 0.012
  };

  joint_names_ = declare_parameter<std::vector<std::string>>("joint_names", default_joint_names);
  merged_trajectory_topic_ = declare_parameter<std::string>(
    "merged_trajectory_topic", "/dual_arm_transport_native/merged_trajectory");
  cartesian_delta_topic_ = declare_parameter<std::string>(
    "cartesian_delta_topic", "/dual_arm_transport_native/cartesian_delta_cmd");
  cartesian_targets_topic_ = declare_parameter<std::string>(
    "cartesian_targets_topic", "/dual_arm_transport_native/cartesian_targets_cmd");
  state6_task_topic_ = declare_parameter<std::string>(
    "state6_task_topic", "/dual_arm_transport_native/state6_task_cmd");
  velocity_command_topic_ = declare_parameter<std::string>(
    "velocity_command_topic", "/dual_joint_group_velocity_controller/commands");
  status_topic_ = declare_parameter<std::string>(
    "status_topic", "/dual_arm_transport_native/status");
  trajectory_action_name_ = declare_parameter<std::string>(
    "trajectory_action_name", "/dual_panda_arm_controller/follow_joint_trajectory");

  trajectory_controller_name_ = declare_parameter<std::string>(
    "trajectory_controller_name", "dual_panda_arm_controller");
  velocity_controller_name_ = declare_parameter<std::string>(
    "velocity_controller_name", "dual_joint_group_velocity_controller");

  use_cascade_joint_pid_ = declare_parameter<bool>("use_cascade_joint_pid", true);
  cascade_pid_auto_switch_controllers_ = declare_parameter<bool>(
    "cascade_pid_auto_switch_controllers", true);

  enable_native_cartesian_delta_ = declare_parameter<bool>("enable_native_cartesian_delta", true);
  enable_native_state6_task_ = declare_parameter<bool>("enable_native_state6_task", true);
  planning_frame_ = declare_parameter<std::string>("planning_frame", "base_link");
  left_group_name_ = declare_parameter<std::string>("left_group_name", "mj_left_arm");
  right_group_name_ = declare_parameter<std::string>("right_group_name", "mj_right_arm");
  left_ee_link_ = declare_parameter<std::string>("left_ee_link", "mj_left_link8");
  right_ee_link_ = declare_parameter<std::string>("right_ee_link", "mj_right_link8");
  cartesian_fraction_threshold_ = declare_parameter<double>("cartesian_fraction_threshold", 0.98);
  cartesian_default_max_step_ = declare_parameter<double>("cartesian_default_max_step", 0.006);
  cartesian_default_target_speed_ = declare_parameter<double>("cartesian_default_target_speed", 0.018);
  merge_default_time_scale_ = declare_parameter<double>("merge_default_time_scale", 1.2);
  merge_min_dt_ = declare_parameter<double>("merge_min_dt", 0.04);
  merge_max_joint_vel_ = declare_parameter<double>("merge_max_joint_vel", 0.30);
  merge_max_joint_acc_ = declare_parameter<double>("merge_max_joint_acc", 0.45);
  merge_vel_lpf_ = declare_parameter<double>("merge_vel_lpf", 0.84);

  cascade_pid_loop_hz_ = declare_parameter<double>("cascade_pid_loop_hz", 120.0);
  cascade_pid_outer_i_clamp_ = declare_parameter<double>("cascade_pid_outer_i_clamp", 0.20);
  cascade_pid_inner_i_clamp_ = declare_parameter<double>("cascade_pid_inner_i_clamp", 0.35);
  cascade_pid_outer_v_limit_ = declare_parameter<double>("cascade_pid_outer_v_limit", 0.70);
  cascade_pid_cmd_max_vel_ = declare_parameter<double>("cascade_pid_cmd_max_vel", 0.85);
  cascade_pid_cmd_max_acc_ = declare_parameter<double>("cascade_pid_cmd_max_acc", 2.20);
  cascade_pid_finish_pos_tol_ = declare_parameter<double>("cascade_pid_finish_pos_tol", 0.008);
  cascade_pid_finish_vel_tol_ = declare_parameter<double>("cascade_pid_finish_vel_tol", 0.050);
  cascade_pid_finish_hold_sec_ = declare_parameter<double>("cascade_pid_finish_hold_sec", 0.12);
  cascade_pid_timeout_pad_sec_ = declare_parameter<double>("cascade_pid_timeout_pad_sec", 2.0);
  action_goal_time_tolerance_sec_ = declare_parameter<double>("action_goal_time_tolerance_sec", 5.0);

  outer_kp_ = expand_joint_vector(
    declare_parameter<std::vector<double>>("cascade_pid_outer_kp", default_outer_kp), 4.2);
  outer_ki_ = expand_joint_vector(
    declare_parameter<std::vector<double>>("cascade_pid_outer_ki", default_outer_ki), 0.06);
  outer_kd_ = expand_joint_vector(
    declare_parameter<std::vector<double>>("cascade_pid_outer_kd", default_outer_kd), 0.11);
  inner_kp_ = expand_joint_vector(
    declare_parameter<std::vector<double>>("cascade_pid_inner_kp", default_inner_kp), 0.85);
  inner_ki_ = expand_joint_vector(
    declare_parameter<std::vector<double>>("cascade_pid_inner_ki", default_inner_ki), 0.12);
  inner_kd_ = expand_joint_vector(
    declare_parameter<std::vector<double>>("cascade_pid_inner_kd", default_inner_kd), 0.020);
}

std::vector<double> DualArmTransportNativeNode::expand_joint_vector(
  const std::vector<double> & raw,
  double fallback) const
{
  const size_t n = joint_names_.size();
  if (n == 0) {
    return {};
  }
  if (raw.size() == n) {
    return raw;
  }
  if (n % 2 == 0 && raw.size() == (n / 2)) {
    std::vector<double> out = raw;
    out.insert(out.end(), raw.begin(), raw.end());
    return out;
  }
  return std::vector<double>(n, fallback);
}

void DualArmTransportNativeNode::joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  if (!msg) {
    return;
  }
  std::lock_guard<std::mutex> lock(state_mutex_);
  for (size_t i = 0; i < msg->name.size(); ++i) {
    const auto it = joint_index_.find(msg->name[i]);
    if (it == joint_index_.end()) {
      continue;
    }
    const size_t idx = it->second;
    if (i < msg->position.size()) {
      current_q_[idx] = msg->position[i];
      joint_seen_[idx] = true;
    }
    if (i < msg->velocity.size()) {
      current_dq_[idx] = msg->velocity[i];
    }
  }
}

void DualArmTransportNativeNode::merged_trajectory_callback(
  const trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
{
  if (!msg) {
    return;
  }
  if (executing_.exchange(true)) {
    RCLCPP_WARN(get_logger(), "Previous trajectory still running, dropping new merged request.");
    return;
  }

  trajectory_msgs::msg::JointTrajectory normalized;
  if (!normalize_trajectory(*msg, normalized)) {
    publish_status("normalize_failed");
    executing_.store(false);
    return;
  }

  std::thread worker([this, normalized]() { execute_trajectory_task(normalized); });
  worker.detach();
}

void DualArmTransportNativeNode::cartesian_delta_callback(
  const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
  if (!enable_native_cartesian_delta_) {
    return;
  }
  if (!msg || msg->data.size() < 3) {
    RCLCPP_WARN(get_logger(), "Cartesian delta cmd requires at least [dx,dy,dz].");
    return;
  }

  const double dx = msg->data[0];
  const double dy = msg->data[1];
  const double dz = msg->data[2];
  const double max_step = (msg->data.size() > 3) ? std::fabs(msg->data[3]) : cartesian_default_max_step_;
  const double target_speed = (msg->data.size() > 4) ? std::fabs(msg->data[4]) : cartesian_default_target_speed_;
  const double time_scale = (msg->data.size() > 5) ? std::fabs(msg->data[5]) : merge_default_time_scale_;

  if (executing_.exchange(true)) {
    RCLCPP_WARN(get_logger(), "Previous trajectory still running, dropping cartesian delta request.");
    return;
  }

  std::thread worker(
    [this, dx, dy, dz, max_step, target_speed, time_scale]() {
      execute_cartesian_delta_task(dx, dy, dz, max_step, target_speed, time_scale);
    });
  worker.detach();
}

void DualArmTransportNativeNode::cartesian_targets_callback(
  const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
  if (!enable_native_cartesian_delta_) {
    return;
  }
  if (!msg || msg->data.size() < 14) {
    RCLCPP_WARN(
      get_logger(),
      "Cartesian targets cmd requires at least 14 values: "
      "[lx,ly,lz,lqx,lqy,lqz,lqw, rx,ry,rz,rqx,rqy,rqz,rqw, ...]");
    return;
  }

  geometry_msgs::msg::Pose left_goal;
  geometry_msgs::msg::Pose right_goal;
  left_goal.position.x = msg->data[0];
  left_goal.position.y = msg->data[1];
  left_goal.position.z = msg->data[2];
  left_goal.orientation.x = msg->data[3];
  left_goal.orientation.y = msg->data[4];
  left_goal.orientation.z = msg->data[5];
  left_goal.orientation.w = msg->data[6];

  right_goal.position.x = msg->data[7];
  right_goal.position.y = msg->data[8];
  right_goal.position.z = msg->data[9];
  right_goal.orientation.x = msg->data[10];
  right_goal.orientation.y = msg->data[11];
  right_goal.orientation.z = msg->data[12];
  right_goal.orientation.w = msg->data[13];

  const double max_step = (msg->data.size() > 14) ? std::fabs(msg->data[14]) : cartesian_default_max_step_;
  const double target_speed = (msg->data.size() > 15) ? std::fabs(msg->data[15]) : cartesian_default_target_speed_;
  const double time_scale = (msg->data.size() > 16) ? std::fabs(msg->data[16]) : merge_default_time_scale_;
  const bool avoid_collisions = (msg->data.size() > 17) ? (msg->data[17] > 0.5) : false;

  if (executing_.exchange(true)) {
    RCLCPP_WARN(get_logger(), "Previous trajectory still running, dropping cartesian targets request.");
    return;
  }

  std::thread worker(
    [this, left_goal, right_goal, max_step, target_speed, time_scale, avoid_collisions]() {
      execute_cartesian_targets_task(
        left_goal, right_goal, max_step, target_speed, time_scale, avoid_collisions);
    });
  worker.detach();
}

void DualArmTransportNativeNode::state6_task_callback(
  const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
  if (!enable_native_state6_task_) {
    return;
  }
  if (!msg || msg->data.size() < 2) {
    RCLCPP_WARN(
      get_logger(),
      "State6 task cmd requires at least [mode, payload...].");
    return;
  }

  const int mode = static_cast<int>(std::llround(msg->data[0]));
  std::vector<double> payload;
  payload.reserve(msg->data.size() - 1);
  for (size_t i = 1; i < msg->data.size(); ++i) {
    payload.push_back(msg->data[i]);
  }

  if (executing_.exchange(true)) {
    RCLCPP_WARN(get_logger(), "Previous trajectory still running, dropping state6 task request.");
    return;
  }

  std::thread worker(
    [this, mode, payload]() {
      execute_state6_task(mode, payload);
    });
  worker.detach();
}

bool DualArmTransportNativeNode::normalize_trajectory(
  const trajectory_msgs::msg::JointTrajectory & in,
  trajectory_msgs::msg::JointTrajectory & out) const
{
  const size_t n = joint_names_.size();
  if (n == 0 || in.points.empty()) {
    RCLCPP_WARN(get_logger(), "normalize_trajectory: empty input.");
    return false;
  }

  std::vector<size_t> index_map(n, static_cast<size_t>(-1));
  for (size_t j = 0; j < n; ++j) {
    for (size_t i = 0; i < in.joint_names.size(); ++i) {
      if (in.joint_names[i] == joint_names_[j]) {
        index_map[j] = i;
        break;
      }
    }
    if (index_map[j] == static_cast<size_t>(-1)) {
      RCLCPP_WARN(get_logger(), "normalize_trajectory: missing joint %s", joint_names_[j].c_str());
      return false;
    }
  }

  out = trajectory_msgs::msg::JointTrajectory();
  out.joint_names = joint_names_;
  out.points.reserve(in.points.size());

  double prev_t = -1e9;
  const double min_dt = 1e-3;

  for (const auto & in_pt : in.points) {
    if (in_pt.positions.size() < in.joint_names.size()) {
      RCLCPP_WARN(get_logger(), "normalize_trajectory: point positions too short.");
      return false;
    }

    trajectory_msgs::msg::JointTrajectoryPoint out_pt;
    out_pt.positions.resize(n, 0.0);
    for (size_t j = 0; j < n; ++j) {
      out_pt.positions[j] = in_pt.positions[index_map[j]];
    }

    if (in_pt.velocities.size() >= in.joint_names.size()) {
      out_pt.velocities.resize(n, 0.0);
      for (size_t j = 0; j < n; ++j) {
        out_pt.velocities[j] = in_pt.velocities[index_map[j]];
      }
    }

    double t = duration_to_sec(in_pt.time_from_start);
    if (out.points.empty()) {
      t = 0.0;
    } else {
      t = std::max(t, prev_t + min_dt);
    }
    out_pt.time_from_start = sec_to_duration(t);
    prev_t = t;
    out.points.push_back(std::move(out_pt));
  }

  return !out.points.empty();
}

bool DualArmTransportNativeNode::derive_reference_velocity(
  const trajectory_msgs::msg::JointTrajectory & traj,
  std::vector<std::vector<double>> & ref_dq) const
{
  const size_t n_pts = traj.points.size();
  const size_t n_joints = joint_names_.size();
  ref_dq.assign(n_pts, std::vector<double>(n_joints, 0.0));
  if (n_pts == 0 || n_joints == 0) {
    return false;
  }

  bool has_velocity = true;
  for (const auto & pt : traj.points) {
    if (pt.velocities.size() != n_joints) {
      has_velocity = false;
      break;
    }
  }

  if (has_velocity) {
    for (size_t i = 0; i < n_pts; ++i) {
      ref_dq[i] = traj.points[i].velocities;
    }
    return true;
  }

  if (n_pts == 1) {
    return true;
  }

  auto point_t = [&](size_t idx) {
    return duration_to_sec(traj.points[idx].time_from_start);
  };

  for (size_t i = 0; i < n_pts; ++i) {
    if (i == 0) {
      const double dt = std::max(1e-6, point_t(1) - point_t(0));
      for (size_t j = 0; j < n_joints; ++j) {
        ref_dq[i][j] = (traj.points[1].positions[j] - traj.points[0].positions[j]) / dt;
      }
    } else if (i + 1 == n_pts) {
      const double dt = std::max(1e-6, point_t(n_pts - 1) - point_t(n_pts - 2));
      for (size_t j = 0; j < n_joints; ++j) {
        ref_dq[i][j] =
          (traj.points[n_pts - 1].positions[j] - traj.points[n_pts - 2].positions[j]) / dt;
      }
    } else {
      const double dt = std::max(1e-6, point_t(i + 1) - point_t(i - 1));
      for (size_t j = 0; j < n_joints; ++j) {
        ref_dq[i][j] =
          (traj.points[i + 1].positions[j] - traj.points[i - 1].positions[j]) / dt;
      }
    }
  }

  return true;
}

bool DualArmTransportNativeNode::sample_reference(
  const std::vector<double> & t_vec,
  const std::vector<std::vector<double>> & q_mat,
  const std::vector<std::vector<double>> & dq_mat,
  double t_query,
  std::vector<double> & q_ref,
  std::vector<double> & dq_ref) const
{
  const size_t n_pts = t_vec.size();
  if (n_pts == 0 || q_mat.size() != n_pts || dq_mat.size() != n_pts) {
    return false;
  }

  if (t_query <= t_vec.front()) {
    q_ref = q_mat.front();
    dq_ref = dq_mat.front();
    return true;
  }
  if (t_query >= t_vec.back()) {
    q_ref = q_mat.back();
    dq_ref = dq_mat.back();
    return true;
  }

  auto it = std::lower_bound(t_vec.begin(), t_vec.end(), t_query);
  size_t i1 = static_cast<size_t>(std::distance(t_vec.begin(), it));
  if (i1 == 0) {
    i1 = 1;
  }
  const size_t i0 = i1 - 1;

  const double t0 = t_vec[i0];
  const double t1 = t_vec[i1];
  const double dt = std::max(1e-9, t1 - t0);
  const double ratio = clamp((t_query - t0) / dt, 0.0, 1.0);

  const size_t n_joints = joint_names_.size();
  q_ref.assign(n_joints, 0.0);
  dq_ref.assign(n_joints, 0.0);

  for (size_t j = 0; j < n_joints; ++j) {
    q_ref[j] = q_mat[i0][j] + ratio * (q_mat[i1][j] - q_mat[i0][j]);
    dq_ref[j] = dq_mat[i0][j] + ratio * (dq_mat[i1][j] - dq_mat[i0][j]);
  }
  return true;
}

bool DualArmTransportNativeNode::read_joint_feedback(std::vector<double> & q, std::vector<double> & dq) const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  for (bool seen : joint_seen_) {
    if (!seen) {
      return false;
    }
  }
  q = current_q_;
  dq = current_dq_;
  return true;
}

void DualArmTransportNativeNode::execute_trajectory_task(const trajectory_msgs::msg::JointTrajectory & traj)
{
  publish_status("executing");
  const bool ok = execute_merged_trajectory_internal(traj);
  publish_status(ok ? "succeeded" : "failed");
  executing_.store(false);
}

void DualArmTransportNativeNode::execute_cartesian_delta_task(
  double dx, double dy, double dz,
  double max_step, double target_speed, double time_scale)
{
  publish_status("executing");
  const bool ok = execute_cartesian_delta_internal(dx, dy, dz, max_step, target_speed, time_scale);
  publish_status(ok ? "succeeded" : "failed");
  executing_.store(false);
}

void DualArmTransportNativeNode::execute_cartesian_targets_task(
  const geometry_msgs::msg::Pose & left_goal,
  const geometry_msgs::msg::Pose & right_goal,
  double max_step, double target_speed, double time_scale,
  bool avoid_collisions)
{
  publish_status("executing");
  const bool ok = execute_cartesian_targets_internal(
    left_goal, right_goal, max_step, target_speed, time_scale, avoid_collisions, -1.0);
  publish_status(ok ? "succeeded" : "failed");
  executing_.store(false);
}

void DualArmTransportNativeNode::execute_state6_task(
  int mode,
  const std::vector<double> & payload)
{
  publish_status("executing");

  bool ok = false;
  if (mode == 0) {
    ok = execute_state6_translate_internal(payload);
  } else if (mode == 1) {
    ok = execute_state6_rotate_internal(payload);
  } else {
    RCLCPP_WARN(get_logger(), "State6 task: unknown mode=%d", mode);
    ok = false;
  }

  publish_status(ok ? "succeeded" : "failed");
  executing_.store(false);
}

bool DualArmTransportNativeNode::execute_merged_trajectory_internal(
  const trajectory_msgs::msg::JointTrajectory & traj)
{
  bool ok = false;
  if (use_cascade_joint_pid_) {
    ok = execute_with_cascade_pid(traj);
    if (!ok) {
      RCLCPP_WARN(get_logger(), "Cascade PID execution failed, fallback to action controller.");
    }
  }
  if (!ok) {
    ok = execute_with_action_fallback(traj);
  }
  return ok;
}

bool DualArmTransportNativeNode::execute_cartesian_delta_internal(
  double dx, double dy, double dz,
  double max_step, double target_speed, double time_scale)
{
  geometry_msgs::msg::Pose left_start;
  geometry_msgs::msg::Pose right_start;
  if (!lookup_current_pose(left_ee_link_, left_start) || !lookup_current_pose(right_ee_link_, right_start)) {
    RCLCPP_WARN(get_logger(), "Cartesian delta: failed to lookup current ee poses.");
    return false;
  }

  geometry_msgs::msg::Pose left_goal = left_start;
  geometry_msgs::msg::Pose right_goal = right_start;
  left_goal.position.x += dx;
  left_goal.position.y += dy;
  left_goal.position.z += dz;
  right_goal.position.x += dx;
  right_goal.position.y += dy;
  right_goal.position.z += dz;

  trajectory_msgs::msg::JointTrajectory left_traj;
  trajectory_msgs::msg::JointTrajectory right_traj;
  double frac_left = 0.0;
  double frac_right = 0.0;

  if (!build_cartesian_traj_for_group(
      left_group_name_, left_ee_link_, left_start, left_goal,
      max_step, false, left_traj, frac_left))
  {
    return false;
  }
  if (!build_cartesian_traj_for_group(
      right_group_name_, right_ee_link_, right_start, right_goal,
      max_step, false, right_traj, frac_right))
  {
    return false;
  }

  if (frac_left < cartesian_fraction_threshold_ || frac_right < cartesian_fraction_threshold_) {
    RCLCPP_WARN(
      get_logger(),
      "Cartesian delta: low fraction left=%.3f right=%.3f threshold=%.3f",
      frac_left, frac_right, cartesian_fraction_threshold_);
    return false;
  }

  trajectory_msgs::msg::JointTrajectory merged;
  const double speed = std::max(0.005, target_speed);
  const double dist = std::sqrt(dx * dx + dy * dy + dz * dz);
  const double desired = std::max(0.8, dist / speed);
  const double raw_total_left = duration_to_sec(left_traj.points.back().time_from_start);
  const double raw_total_right = duration_to_sec(right_traj.points.back().time_from_start);
  const double raw_total = std::max(0.1, std::max(raw_total_left, raw_total_right));
  const double auto_time_scale = std::max(1.0, desired / raw_total);
  const double merge_time_scale = std::max(0.5, std::max(time_scale, auto_time_scale));

  if (!build_merged_trajectory_from_lr(left_traj, right_traj, merge_time_scale, merged)) {
    return false;
  }

  RCLCPP_INFO(
    get_logger(),
    "Cartesian delta execute: d=(%.4f, %.4f, %.4f), fraction=(%.3f, %.3f), points=%zu, tscale=%.2f",
    dx, dy, dz, frac_left, frac_right, merged.points.size(), merge_time_scale);

  return execute_merged_trajectory_internal(merged);
}

bool DualArmTransportNativeNode::execute_cartesian_targets_internal(
  const geometry_msgs::msg::Pose & left_goal,
  const geometry_msgs::msg::Pose & right_goal,
  double max_step, double target_speed, double time_scale,
  bool avoid_collisions,
  double fraction_threshold_override)
{
  geometry_msgs::msg::Pose left_start;
  geometry_msgs::msg::Pose right_start;
  if (!lookup_current_pose(left_ee_link_, left_start) || !lookup_current_pose(right_ee_link_, right_start)) {
    RCLCPP_WARN(get_logger(), "Cartesian targets: failed to lookup current ee poses.");
    return false;
  }

  trajectory_msgs::msg::JointTrajectory left_traj;
  trajectory_msgs::msg::JointTrajectory right_traj;
  double frac_left = 0.0;
  double frac_right = 0.0;

  if (!build_cartesian_traj_for_group(
      left_group_name_, left_ee_link_, left_start, left_goal,
      max_step, avoid_collisions, left_traj, frac_left))
  {
    return false;
  }
  if (!build_cartesian_traj_for_group(
      right_group_name_, right_ee_link_, right_start, right_goal,
      max_step, avoid_collisions, right_traj, frac_right))
  {
    return false;
  }

  const double fraction_threshold = (
    fraction_threshold_override > 0.0
  ) ? clamp(fraction_threshold_override, 0.50, 0.999) : cartesian_fraction_threshold_;
  if (frac_left < fraction_threshold || frac_right < fraction_threshold) {
    RCLCPP_WARN(
      get_logger(),
      "Cartesian targets: low fraction left=%.3f right=%.3f threshold=%.3f",
      frac_left, frac_right, fraction_threshold);
    return false;
  }

  trajectory_msgs::msg::JointTrajectory merged;

  const double left_dx = left_goal.position.x - left_start.position.x;
  const double left_dy = left_goal.position.y - left_start.position.y;
  const double left_dz = left_goal.position.z - left_start.position.z;
  const double right_dx = right_goal.position.x - right_start.position.x;
  const double right_dy = right_goal.position.y - right_start.position.y;
  const double right_dz = right_goal.position.z - right_start.position.z;
  const double left_dist = std::sqrt(left_dx * left_dx + left_dy * left_dy + left_dz * left_dz);
  const double right_dist = std::sqrt(right_dx * right_dx + right_dy * right_dy + right_dz * right_dz);
  const double dist = std::max(left_dist, right_dist);
  const double speed = std::max(0.005, target_speed);
  const double desired = std::max(0.8, dist / speed);
  const double raw_total_left = duration_to_sec(left_traj.points.back().time_from_start);
  const double raw_total_right = duration_to_sec(right_traj.points.back().time_from_start);
  const double raw_total = std::max(0.1, std::max(raw_total_left, raw_total_right));
  const double auto_time_scale = std::max(1.0, desired / raw_total);
  const double merge_time_scale = std::max(0.5, std::max(time_scale, auto_time_scale));

  if (!build_merged_trajectory_from_lr(left_traj, right_traj, merge_time_scale, merged)) {
    return false;
  }

  RCLCPP_INFO(
    get_logger(),
    "Cartesian targets execute: ldist=%.4f rdist=%.4f fraction=(%.3f, %.3f), points=%zu, tscale=%.2f",
    left_dist, right_dist, frac_left, frac_right, merged.points.size(), merge_time_scale);

  return execute_merged_trajectory_internal(merged);
}

bool DualArmTransportNativeNode::execute_state6_translate_internal(const std::vector<double> & payload)
{
  // payload:
  // [lift_dz, transport_dx, transport_dy, transport_dz,
  //  max_step?, lift_speed?, transport_speed?, time_scale?,
  //  transport_one_shot_segment?, transport_retry_segment?]
  if (payload.size() < 4) {
    RCLCPP_WARN(
      get_logger(),
      "State6 translate payload too short: need >=4, got=%zu", payload.size());
    return false;
  }

  const double lift_dz = payload[0];
  const double dx = payload[1];
  const double dy = payload[2];
  const double dz = payload[3];

  const double max_step = (payload.size() > 4) ?
    std::fabs(payload[4]) : cartesian_default_max_step_;
  const double lift_speed = (payload.size() > 5) ?
    std::fabs(payload[5]) : cartesian_default_target_speed_;
  const double transport_speed = (payload.size() > 6) ?
    std::fabs(payload[6]) : cartesian_default_target_speed_;
  const double time_scale = (payload.size() > 7) ?
    std::fabs(payload[7]) : merge_default_time_scale_;
  const double transport_one_shot_segment_in = (payload.size() > 8) ?
    std::max(0.02, std::fabs(payload[8])) : -1.0;
  const double transport_retry_segment = (payload.size() > 9) ?
    std::max(0.02, std::fabs(payload[9])) : 0.08;

  RCLCPP_INFO(
    get_logger(),
    "State6 translate start: lift=%.4f, d=(%.4f, %.4f, %.4f), max_step=%.4f, "
    "lift_speed=%.4f, transport_speed=%.4f, seg_one_shot=%.4f, seg_retry=%.4f",
    lift_dz, dx, dy, dz, max_step, lift_speed, transport_speed,
    transport_one_shot_segment_in, transport_retry_segment);

  if (std::fabs(lift_dz) > 1e-5) {
    const bool ok_lift = execute_cartesian_delta_internal(
      0.0, 0.0, lift_dz, max_step, lift_speed, time_scale);
    if (!ok_lift) {
      // 与 Python 流程保持一致：抬升失败时记录告警，但继续尝试主搬运。
      RCLCPP_WARN(get_logger(), "State6 translate: lift step failed, continue transport.");
    }
  }

  const double dist = std::sqrt(dx * dx + dy * dy + dz * dz);
  if (dist <= 1e-6) {
    RCLCPP_INFO(get_logger(), "State6 translate: transport delta near zero, done.");
    return true;
  }

  const double max_axis = std::max(std::fabs(dx), std::max(std::fabs(dy), std::fabs(dz)));
  const double transport_one_shot_segment = (transport_one_shot_segment_in > 0.0) ?
    transport_one_shot_segment_in : (max_axis + 0.02);

  auto run_segmented = [&](double segment_limit, double speed) {
      const int seg_count = std::max(1, static_cast<int>(std::ceil(max_axis / std::max(0.02, segment_limit))));
      const double step_dx = dx / static_cast<double>(seg_count);
      const double step_dy = dy / static_cast<double>(seg_count);
      const double step_dz = dz / static_cast<double>(seg_count);

      for (int i = 0; i < seg_count; ++i) {
        const bool ok_segment = execute_cartesian_delta_internal(
          step_dx, step_dy, step_dz, max_step, speed, time_scale);
        if (!ok_segment) {
          RCLCPP_WARN(
            get_logger(),
            "State6 translate segment failed: %d/%d (seg_limit=%.4f)",
            i + 1, seg_count, segment_limit);
          return false;
        }
      }
      return true;
    };

  if (run_segmented(transport_one_shot_segment, transport_speed)) {
    return true;
  }

  const double retry_speed = std::max(0.005, transport_speed * 0.9);
  RCLCPP_WARN(
    get_logger(),
    "State6 translate one-shot path failed, retry segmented path: seg=%.4f speed=%.4f",
    transport_retry_segment, retry_speed);
  return run_segmented(transport_retry_segment, retry_speed);
}

bool DualArmTransportNativeNode::execute_state6_rotate_internal(const std::vector<double> & payload)
{
  // payload:
  // [center_x, center_y, total_deg, step_limit_deg, rotate_orientation,
  //  max_step?, target_speed?, time_scale?,
  //  try_without_orientation?, avoid_collisions?, fraction_threshold?, settle_sec?]
  if (payload.size() < 5) {
    RCLCPP_WARN(
      get_logger(),
      "State6 rotate payload too short: need >=5, got=%zu", payload.size());
    return false;
  }

  const double center_x = payload[0];
  const double center_y = payload[1];
  const double total_deg = payload[2];
  const double step_limit_deg = std::max(2.0, std::fabs(payload[3]));
  const bool rotate_orientation = (payload[4] > 0.5);
  const double max_step = (payload.size() > 5) ?
    std::fabs(payload[5]) : cartesian_default_max_step_;
  const double target_speed = (payload.size() > 6) ?
    std::fabs(payload[6]) : cartesian_default_target_speed_;
  const double time_scale = (payload.size() > 7) ?
    std::fabs(payload[7]) : merge_default_time_scale_;
  const bool try_without_orientation = (payload.size() > 8) ?
    (payload[8] > 0.5) : false;
  const bool avoid_collisions = (payload.size() > 9) ?
    (payload[9] > 0.5) : false;
  const double fraction_threshold_override = (payload.size() > 10) ?
    clamp(payload[10], 0.50, 0.999) : -1.0;
  const double settle_sec = (payload.size() > 11) ?
    std::max(0.0, payload[11]) : 0.0;

  if (std::fabs(total_deg) < 0.2) {
    RCLCPP_INFO(get_logger(), "State6 rotate: total angle near zero, skip.");
    return true;
  }

  const int step_count = std::max(
    1, static_cast<int>(std::ceil(std::fabs(total_deg) / step_limit_deg)));
  const double step_deg = total_deg / static_cast<double>(step_count);
  const double kPi = 3.14159265358979323846;
  const double step_rad = step_deg * kPi / 180.0;

  RCLCPP_INFO(
    get_logger(),
    "State6 rotate start: center=(%.4f, %.4f), total=%.2f deg, step=%.2f deg x %d, "
    "rotate_orientation=%s, try_wo_ori=%s, avoid_collisions=%s, frac_override=%.3f, "
    "max_step=%.4f, speed=%.4f",
    center_x, center_y, total_deg, step_deg, step_count,
    rotate_orientation ? "true" : "false",
    try_without_orientation ? "true" : "false",
    avoid_collisions ? "true" : "false",
    fraction_threshold_override,
    max_step, target_speed);

  auto run_step = [&](double delta_rad, bool apply_orientation) {
      geometry_msgs::msg::Pose left_start;
      geometry_msgs::msg::Pose right_start;
      if (!lookup_current_pose(left_ee_link_, left_start) || !lookup_current_pose(right_ee_link_, right_start)) {
        RCLCPP_WARN(get_logger(), "State6 rotate step: failed to lookup current ee poses.");
        return false;
      }

      const double c = std::cos(delta_rad);
      const double s = std::sin(delta_rad);

      geometry_msgs::msg::Pose left_goal = left_start;
      geometry_msgs::msg::Pose right_goal = right_start;

      const double lx = left_start.position.x - center_x;
      const double ly = left_start.position.y - center_y;
      const double rx = right_start.position.x - center_x;
      const double ry = right_start.position.y - center_y;

      left_goal.position.x = center_x + c * lx - s * ly;
      left_goal.position.y = center_y + s * lx + c * ly;
      right_goal.position.x = center_x + c * rx - s * ry;
      right_goal.position.y = center_y + s * rx + c * ry;

      if (apply_orientation) {
        left_goal.orientation = rotate_quaternion_about_z(left_start.orientation, delta_rad);
        right_goal.orientation = rotate_quaternion_about_z(right_start.orientation, delta_rad);
      }

      return execute_cartesian_targets_internal(
        left_goal, right_goal, max_step, target_speed, time_scale,
        avoid_collisions, fraction_threshold_override);
    };

  auto attempt_step = [&](double delta_rad) {
      bool ok = run_step(delta_rad, rotate_orientation);
      if (!ok && rotate_orientation && try_without_orientation) {
        RCLCPP_WARN(
          get_logger(),
          "State6 rotate step strict-orientation failed, retry without orientation.");
        ok = run_step(delta_rad, false);
      }
      return ok;
    };

  for (int i = 0; i < step_count; ++i) {
    bool ok_step = attempt_step(step_rad);
    if (!ok_step && std::fabs(step_deg) > 2.5) {
      const double half = 0.5 * step_rad;
      RCLCPP_WARN(
        get_logger(),
        "State6 rotate step failed (%d/%d), retry with half-step twice.",
        i + 1, step_count);
      ok_step = attempt_step(half) && attempt_step(half);
    }

    if (!ok_step) {
      RCLCPP_WARN(get_logger(), "State6 rotate failed at step %d/%d.", i + 1, step_count);
      return false;
    }
    if (settle_sec > 1e-4) {
      std::this_thread::sleep_for(std::chrono::duration<double>(settle_sec));
    }
  }

  return true;
}

bool DualArmTransportNativeNode::lookup_current_pose(
  const std::string & link_name,
  geometry_msgs::msg::Pose & pose_out) const
{
  if (!tf_buffer_) {
    return false;
  }
  try {
    const auto tf_msg = tf_buffer_->lookupTransform(
      planning_frame_, link_name, tf2::TimePointZero, tf2::durationFromSec(0.15));
    pose_out.position.x = tf_msg.transform.translation.x;
    pose_out.position.y = tf_msg.transform.translation.y;
    pose_out.position.z = tf_msg.transform.translation.z;
    pose_out.orientation = tf_msg.transform.rotation;
    return true;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(get_logger(), "lookup_current_pose failed for %s: %s", link_name.c_str(), ex.what());
    return false;
  }
}

bool DualArmTransportNativeNode::build_cartesian_traj_for_group(
  const std::string & group_name,
  const std::string & link_name,
  const geometry_msgs::msg::Pose & start_pose,
  const geometry_msgs::msg::Pose & goal_pose,
  double max_step,
  bool avoid_collisions,
  trajectory_msgs::msg::JointTrajectory & traj_out,
  double & fraction_out)
{
  if (!cartesian_client_ || !cartesian_client_->wait_for_service(1s)) {
    RCLCPP_WARN(get_logger(), "GetCartesianPath service unavailable.");
    return false;
  }

  auto req = std::make_shared<moveit_msgs::srv::GetCartesianPath::Request>();
  req->header.frame_id = planning_frame_;
  req->group_name = group_name;
  req->link_name = link_name;
  req->max_step = std::max(0.001, max_step);
  req->jump_threshold = 0.0;
  req->avoid_collisions = avoid_collisions;
  req->waypoints.push_back(start_pose);
  req->waypoints.push_back(goal_pose);

  auto future = cartesian_client_->async_send_request(req);
  if (future.wait_for(12s) != std::future_status::ready) {
    RCLCPP_WARN(get_logger(), "GetCartesianPath timeout for group %s.", group_name.c_str());
    return false;
  }

  const auto resp = future.get();
  if (!resp) {
    return false;
  }

  fraction_out = resp->fraction;
  traj_out = resp->solution.joint_trajectory;
  if (traj_out.points.empty()) {
    RCLCPP_WARN(get_logger(), "GetCartesianPath returned empty trajectory for group %s.", group_name.c_str());
    return false;
  }
  return true;
}

bool DualArmTransportNativeNode::build_merged_trajectory_from_lr(
  const trajectory_msgs::msg::JointTrajectory & left_traj,
  const trajectory_msgs::msg::JointTrajectory & right_traj,
  double time_scale,
  trajectory_msgs::msg::JointTrajectory & merged_out) const
{
  const size_t n_joints = joint_names_.size();
  if (n_joints == 0 || n_joints % 2 != 0) {
    return false;
  }
  if (left_traj.points.empty() || right_traj.points.empty()) {
    return false;
  }

  const size_t n_arm = n_joints / 2;
  std::vector<std::string> left_expected(joint_names_.begin(), joint_names_.begin() + static_cast<long>(n_arm));
  std::vector<std::string> right_expected(joint_names_.begin() + static_cast<long>(n_arm), joint_names_.end());

  auto build_map = [](const std::vector<std::string> & source, const std::vector<std::string> & target,
      std::vector<size_t> & out_map) {
      out_map.assign(target.size(), static_cast<size_t>(-1));
      for (size_t j = 0; j < target.size(); ++j) {
        for (size_t i = 0; i < source.size(); ++i) {
          if (source[i] == target[j]) {
            out_map[j] = i;
            break;
          }
        }
        if (out_map[j] == static_cast<size_t>(-1)) {
          return false;
        }
      }
      return true;
    };

  std::vector<size_t> map_left;
  std::vector<size_t> map_right;
  if (!build_map(left_traj.joint_names, left_expected, map_left)) {
    return false;
  }
  if (!build_map(right_traj.joint_names, right_expected, map_right)) {
    return false;
  }

  auto point_t = [&](const trajectory_msgs::msg::JointTrajectoryPoint & pt) {
      return duration_to_sec(pt.time_from_start);
    };

  std::set<double> t_set;
  t_set.insert(0.0);
  for (const auto & pt : left_traj.points) {
    t_set.insert(std::max(0.0, point_t(pt)));
  }
  for (const auto & pt : right_traj.points) {
    t_set.insert(std::max(0.0, point_t(pt)));
  }

  std::vector<double> t_unified;
  t_unified.reserve(t_set.size());
  for (double t_raw : t_set) {
    t_unified.push_back(t_raw * std::max(0.1, time_scale));
  }

  const double min_dt = std::max(0.01, merge_min_dt_);
  for (size_t i = 1; i < t_unified.size(); ++i) {
    t_unified[i] = std::max(t_unified[i], t_unified[i - 1] + min_dt);
  }

  auto extract_positions = [](const trajectory_msgs::msg::JointTrajectoryPoint & pt,
      const std::vector<size_t> & index_map,
      std::vector<double> & out) {
      out.resize(index_map.size(), 0.0);
      for (size_t i = 0; i < index_map.size(); ++i) {
        const size_t src = index_map[i];
        if (src < pt.positions.size()) {
          out[i] = pt.positions[src];
        }
      }
    };

  auto interp_arm = [&](const trajectory_msgs::msg::JointTrajectory & traj,
      const std::vector<size_t> & index_map,
      double t_scaled,
      std::vector<double> & out) {
      const double query_t = t_scaled / std::max(0.1, time_scale);
      if (query_t <= point_t(traj.points.front())) {
        extract_positions(traj.points.front(), index_map, out);
        return;
      }
      if (query_t >= point_t(traj.points.back())) {
        extract_positions(traj.points.back(), index_map, out);
        return;
      }
      for (size_t i = 0; i + 1 < traj.points.size(); ++i) {
        const double t1 = point_t(traj.points[i]);
        const double t2 = point_t(traj.points[i + 1]);
        if (t1 <= query_t && query_t <= t2) {
          const double ratio = (query_t - t1) / std::max(1e-9, t2 - t1);
          std::vector<double> p1;
          std::vector<double> p2;
          extract_positions(traj.points[i], index_map, p1);
          extract_positions(traj.points[i + 1], index_map, p2);
          out.resize(index_map.size(), 0.0);
          for (size_t k = 0; k < out.size(); ++k) {
            out[k] = p1[k] + ratio * (p2[k] - p1[k]);
          }
          return;
        }
      }
      extract_positions(traj.points.back(), index_map, out);
    };

  merged_out = trajectory_msgs::msg::JointTrajectory();
  merged_out.joint_names = joint_names_;
  merged_out.points.reserve(t_unified.size());

  for (double t : t_unified) {
    std::vector<double> left_pos;
    std::vector<double> right_pos;
    interp_arm(left_traj, map_left, t, left_pos);
    interp_arm(right_traj, map_right, t, right_pos);

    trajectory_msgs::msg::JointTrajectoryPoint pt;
    pt.positions.reserve(n_joints);
    pt.positions.insert(pt.positions.end(), left_pos.begin(), left_pos.end());
    pt.positions.insert(pt.positions.end(), right_pos.begin(), right_pos.end());
    pt.time_from_start = sec_to_duration(t);
    merged_out.points.push_back(std::move(pt));
  }

  // 平滑速度估计，抑制 stop-go
  const size_t n_pts = merged_out.points.size();
  const double max_vel = std::max(0.05, merge_max_joint_vel_);
  const double max_acc = std::max(0.05, merge_max_joint_acc_);
  const double vel_alpha = clamp(merge_vel_lpf_, 0.0, 0.98);

  std::vector<double> prev_vel(n_joints, 0.0);
  for (size_t i = 0; i < n_pts; ++i) {
    std::vector<double> raw(n_joints, 0.0);
    if (n_pts > 2 && i > 0 && i + 1 < n_pts) {
      const double dt = std::max(1e-6, t_unified[i + 1] - t_unified[i - 1]);
      for (size_t j = 0; j < n_joints; ++j) {
        raw[j] = (merged_out.points[i + 1].positions[j] - merged_out.points[i - 1].positions[j]) / dt;
      }
    }

    std::vector<double> vel(n_joints, 0.0);
    for (size_t j = 0; j < n_joints; ++j) {
      vel[j] = vel_alpha * prev_vel[j] + (1.0 - vel_alpha) * raw[j];
      vel[j] = clamp(vel[j], -max_vel, max_vel);
    }

    if (i > 0) {
      const double dt_prev = std::max(1e-6, t_unified[i] - t_unified[i - 1]);
      const double max_dv = max_acc * dt_prev;
      for (size_t j = 0; j < n_joints; ++j) {
        vel[j] = prev_vel[j] + clamp(vel[j] - prev_vel[j], -max_dv, max_dv);
      }
    }

    merged_out.points[i].velocities = vel;
    prev_vel = vel;
  }

  if (!merged_out.points.empty()) {
    merged_out.points.front().velocities.assign(n_joints, 0.0);
    merged_out.points.back().velocities.assign(n_joints, 0.0);
  }

  return true;
}

bool DualArmTransportNativeNode::execute_with_cascade_pid(
  const trajectory_msgs::msg::JointTrajectory & traj)
{
  const size_t n_joints = joint_names_.size();
  if (n_joints == 0 || traj.points.size() < 2) {
    return false;
  }

  bool switched = false;
  if (cascade_pid_auto_switch_controllers_) {
    if (!switch_to_velocity_controller()) {
      return false;
    }
    switched = true;
  }

  std::vector<double> t_vec;
  std::vector<std::vector<double>> q_mat;
  std::vector<std::vector<double>> dq_mat;

  t_vec.reserve(traj.points.size());
  q_mat.reserve(traj.points.size());
  for (const auto & pt : traj.points) {
    t_vec.push_back(duration_to_sec(pt.time_from_start));
    q_mat.push_back(pt.positions);
  }
  if (!derive_reference_velocity(traj, dq_mat)) {
    if (switched) {
      switch_to_trajectory_controller();
    }
    return false;
  }

  const double total_sec = t_vec.back();

  auto ready_deadline = std::chrono::steady_clock::now() + 1s;
  std::vector<double> q_meas;
  std::vector<double> dq_meas;
  while (rclcpp::ok() && !read_joint_feedback(q_meas, dq_meas)) {
    if (std::chrono::steady_clock::now() > ready_deadline) {
      RCLCPP_WARN(get_logger(), "Cascade PID: joint feedback not ready.");
      if (switched) {
        switch_to_trajectory_controller();
      }
      return false;
    }
    std::this_thread::sleep_for(10ms);
  }

  std::vector<CascadePidJointGains> gains(n_joints);
  std::vector<CascadePidJointState> states(n_joints);
  for (size_t i = 0; i < n_joints; ++i) {
    gains[i].outer_kp = outer_kp_[i];
    gains[i].outer_ki = outer_ki_[i];
    gains[i].outer_kd = outer_kd_[i];
    gains[i].inner_kp = inner_kp_[i];
    gains[i].inner_ki = inner_ki_[i];
    gains[i].inner_kd = inner_kd_[i];
  }
  cascade_pid_reset_all(states.data(), static_cast<int>(states.size()));

  const double loop_hz = std::max(60.0, cascade_pid_loop_hz_);
  const double finish_pos_tol = std::max(0.001, cascade_pid_finish_pos_tol_);
  const double finish_vel_tol = std::max(0.001, cascade_pid_finish_vel_tol_);
  const double finish_hold_sec = std::max(0.02, cascade_pid_finish_hold_sec_);
  const double timeout_pad_sec = std::max(0.5, cascade_pid_timeout_pad_sec_);

  rclcpp::Rate rate(loop_hz);
  const auto start_tp = std::chrono::steady_clock::now();
  auto prev_tp = start_tp;
  double finish_hold = 0.0;

  std::vector<double> q_ref(n_joints, 0.0);
  std::vector<double> dq_ref(n_joints, 0.0);
  std::vector<double> cmd(n_joints, 0.0);

  bool ok = false;

  while (rclcpp::ok()) {
    const auto now_tp = std::chrono::steady_clock::now();
    const std::chrono::duration<double> elapsed_dur = now_tp - start_tp;
    const std::chrono::duration<double> dt_dur = now_tp - prev_tp;
    const double elapsed = elapsed_dur.count();
    const double dt = clamp(dt_dur.count(), 1e-3, 0.05);
    prev_tp = now_tp;

    if (elapsed > total_sec + timeout_pad_sec) {
      RCLCPP_WARN(
        get_logger(), "Cascade PID timeout elapsed=%.3f limit=%.3f",
        elapsed, total_sec + timeout_pad_sec);
      ok = false;
      break;
    }

    if (!read_joint_feedback(q_meas, dq_meas)) {
      RCLCPP_WARN(get_logger(), "Cascade PID lost joint feedback.");
      ok = false;
      break;
    }

    const double query_t = std::min(std::max(0.0, elapsed), total_sec);
    if (!sample_reference(t_vec, q_mat, dq_mat, query_t, q_ref, dq_ref)) {
      ok = false;
      break;
    }

    cascade_pid_step_all(
      gains.data(),
      states.data(),
      static_cast<int>(n_joints),
      q_ref.data(),
      dq_ref.data(),
      q_meas.data(),
      dq_meas.data(),
      dt,
      cascade_pid_outer_i_clamp_,
      cascade_pid_inner_i_clamp_,
      cascade_pid_outer_v_limit_,
      cascade_pid_cmd_max_vel_,
      cascade_pid_cmd_max_acc_,
      cmd.data());

    publish_velocity_command(cmd);

    if (elapsed >= total_sec) {
      double max_pos_err = 0.0;
      double max_vel = 0.0;
      for (size_t i = 0; i < n_joints; ++i) {
        max_pos_err = std::max(max_pos_err, std::fabs(q_ref[i] - q_meas[i]));
        max_vel = std::max(max_vel, std::fabs(dq_meas[i]));
      }

      if (max_pos_err <= finish_pos_tol && max_vel <= finish_vel_tol) {
        finish_hold += dt;
      } else {
        finish_hold = 0.0;
      }

      if (finish_hold >= finish_hold_sec) {
        RCLCPP_INFO(
          get_logger(), "Cascade PID done: max_pos_err=%.5f, max_vel=%.5f", max_pos_err, max_vel);
        ok = true;
        break;
      }
    }

    rate.sleep();
  }

  publish_velocity_command(std::vector<double>(n_joints, 0.0));
  std::this_thread::sleep_for(10ms);
  publish_velocity_command(std::vector<double>(n_joints, 0.0));

  if (switched) {
    switch_to_trajectory_controller();
  }

  return ok;
}

bool DualArmTransportNativeNode::execute_with_action_fallback(
  const trajectory_msgs::msg::JointTrajectory & traj)
{
  if (!traj_action_client_->wait_for_action_server(2s)) {
    RCLCPP_WARN(get_logger(), "Trajectory action server not ready: %s", trajectory_action_name_.c_str());
    return false;
  }

  Fjt::Goal goal;
  goal.trajectory = traj;
  goal.goal_time_tolerance = sec_to_duration(action_goal_time_tolerance_sec_);

  auto goal_future = traj_action_client_->async_send_goal(goal);
  if (goal_future.wait_for(5s) != std::future_status::ready) {
    RCLCPP_WARN(get_logger(), "Action send goal timeout.");
    return false;
  }

  auto goal_handle = goal_future.get();
  if (!goal_handle) {
    RCLCPP_WARN(get_logger(), "Action goal rejected.");
    return false;
  }

  const double total_sec = duration_to_sec(traj.points.back().time_from_start);
  const auto timeout = std::chrono::duration<double>(std::max(30.0, total_sec + 20.0));

  auto result_future = traj_action_client_->async_get_result(goal_handle);
  if (result_future.wait_for(timeout) != std::future_status::ready) {
    RCLCPP_WARN(get_logger(), "Action result timeout.");
    return false;
  }

  const auto wrapped_result = result_future.get();
  const bool ok = wrapped_result.code == rclcpp_action::ResultCode::SUCCEEDED;
  RCLCPP_INFO(get_logger(), "Action fallback result: %s", ok ? "SUCCEEDED" : "FAILED");
  return ok;
}

std::unordered_map<std::string, std::string> DualArmTransportNativeNode::list_controllers()
{
  std::unordered_map<std::string, std::string> out;
  if (!list_ctrl_client_->wait_for_service(1s)) {
    return out;
  }

  auto req = std::make_shared<controller_manager_msgs::srv::ListControllers::Request>();
  auto future = list_ctrl_client_->async_send_request(req);
  if (future.wait_for(2s) != std::future_status::ready) {
    return out;
  }

  auto resp = future.get();
  if (!resp) {
    return out;
  }
  for (const auto & c : resp->controller) {
    out[c.name] = c.state;
  }
  return out;
}

bool DualArmTransportNativeNode::switch_controllers(
  const std::vector<std::string> & activate,
  const std::vector<std::string> & deactivate)
{
  if (activate.empty() && deactivate.empty()) {
    return true;
  }
  if (!switch_ctrl_client_->wait_for_service(1s)) {
    return false;
  }

  auto req = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
  req->activate_controllers = activate;
  req->deactivate_controllers = deactivate;
  req->strictness = 1;

  auto future = switch_ctrl_client_->async_send_request(req);
  if (future.wait_for(3s) != std::future_status::ready) {
    return false;
  }

  auto resp = future.get();
  if (!resp) {
    return false;
  }
  return resp->ok;
}

bool DualArmTransportNativeNode::switch_to_velocity_controller()
{
  const auto ctrls = list_controllers();
  if (ctrls.empty()) {
    return false;
  }

  auto it_vel = ctrls.find(velocity_controller_name_);
  if (it_vel == ctrls.end()) {
    RCLCPP_WARN(get_logger(), "Velocity controller not found: %s", velocity_controller_name_.c_str());
    return false;
  }

  std::vector<std::string> activate;
  std::vector<std::string> deactivate;

  if (it_vel->second != "active") {
    activate.push_back(velocity_controller_name_);
  }

  auto it_traj = ctrls.find(trajectory_controller_name_);
  if (it_traj != ctrls.end() && it_traj->second == "active") {
    deactivate.push_back(trajectory_controller_name_);
  }

  const bool ok = switch_controllers(activate, deactivate);
  if (!ok) {
    RCLCPP_WARN(get_logger(), "Failed switching to velocity controller.");
  }
  return ok;
}

bool DualArmTransportNativeNode::switch_to_trajectory_controller()
{
  const auto ctrls = list_controllers();
  if (ctrls.empty()) {
    return false;
  }

  std::vector<std::string> activate;
  std::vector<std::string> deactivate;

  auto it_traj = ctrls.find(trajectory_controller_name_);
  if (it_traj != ctrls.end() && it_traj->second != "active") {
    activate.push_back(trajectory_controller_name_);
  }

  auto it_vel = ctrls.find(velocity_controller_name_);
  if (it_vel != ctrls.end() && it_vel->second == "active") {
    deactivate.push_back(velocity_controller_name_);
  }

  const bool ok = switch_controllers(activate, deactivate);
  if (!ok) {
    RCLCPP_WARN(get_logger(), "Failed switching back to trajectory controller.");
  }
  return ok;
}

void DualArmTransportNativeNode::publish_velocity_command(const std::vector<double> & cmd)
{
  std_msgs::msg::Float64MultiArray msg;
  msg.data = cmd;
  joint_vel_pub_->publish(msg);
}

void DualArmTransportNativeNode::publish_status(const std::string & text) const
{
  std_msgs::msg::String msg;
  msg.data = text;
  status_pub_->publish(msg);
}

double DualArmTransportNativeNode::duration_to_sec(const builtin_interfaces::msg::Duration & d)
{
  return static_cast<double>(d.sec) + static_cast<double>(d.nanosec) * 1e-9;
}

builtin_interfaces::msg::Duration DualArmTransportNativeNode::sec_to_duration(double sec)
{
  builtin_interfaces::msg::Duration d;
  if (sec < 0.0) {
    sec = 0.0;
  }
  d.sec = static_cast<int32_t>(std::floor(sec));
  d.nanosec = static_cast<uint32_t>((sec - static_cast<double>(d.sec)) * 1e9);
  return d;
}

double DualArmTransportNativeNode::clamp(double v, double lo, double hi)
{
  if (v < lo) {
    return lo;
  }
  if (v > hi) {
    return hi;
  }
  return v;
}

}  // namespace dual_arm_transport_native
