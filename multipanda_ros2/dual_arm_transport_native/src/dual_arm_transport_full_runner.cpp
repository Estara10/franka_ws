#include <fcntl.h>
#include <signal.h>
#include <sys/wait.h>
#include <unistd.h>

#include <chrono>
#include <cstring>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "controller_manager_msgs/srv/list_controllers.hpp"
#include "moveit_msgs/action/move_group.hpp"
#include "moveit_msgs/srv/get_position_ik.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/string.hpp"

namespace dual_arm_transport_native
{

class DualArmTransportFullRunner : public rclcpp::Node
{
public:
  DualArmTransportFullRunner()
  : Node("dual_arm_transport_full_runner")
  {
    start_sim_ = declare_parameter<bool>("start_sim", true);
    start_moveit_ = declare_parameter<bool>("start_moveit", true);
    start_native_executor_ = declare_parameter<bool>("start_native_executor", true);
    auto_shutdown_ = declare_parameter<bool>("auto_shutdown", true);

    sim_log_path_ = declare_parameter<std::string>("sim_log_path", "/tmp/dual_franka_sim_cpp.log");
    moveit_log_path_ = declare_parameter<std::string>("moveit_log_path", "/tmp/dual_moveit_cpp.log");
    native_log_path_ = declare_parameter<std::string>(
      "native_log_path", "/tmp/dual_arm_transport_native_cpp.log");

    state6_topic_ = declare_parameter<std::string>(
      "state6_topic", "/dual_arm_transport_native/state6_task_cmd");
    status_topic_ = declare_parameter<std::string>(
      "status_topic", "/dual_arm_transport_native/status");

    startup_timeout_sec_ = declare_parameter<double>("startup_timeout_sec", 90.0);
    native_result_timeout_sec_ = declare_parameter<double>("native_result_timeout_sec", 120.0);

    task_mode_ = declare_parameter<std::string>("task_mode", "translate");

    // translate mode params
    trans_lift_dz_ = declare_parameter<double>("translate.lift_dz", 0.02);
    trans_dx_ = declare_parameter<double>("translate.dx", 0.10);
    trans_dy_ = declare_parameter<double>("translate.dy", 0.00);
    trans_dz_ = declare_parameter<double>("translate.dz", 0.00);
    trans_max_step_ = declare_parameter<double>("translate.max_step", 0.006);
    trans_lift_speed_ = declare_parameter<double>("translate.lift_speed", 0.020);
    trans_transport_speed_ = declare_parameter<double>("translate.transport_speed", 0.018);
    trans_time_scale_ = declare_parameter<double>("translate.time_scale", 1.2);
    trans_one_shot_segment_ = declare_parameter<double>("translate.one_shot_segment", 0.16);
    trans_retry_segment_ = declare_parameter<double>("translate.retry_segment", 0.08);

    // rotate mode params
    rot_center_x_ = declare_parameter<double>("rotate.center_x", 0.50);
    rot_center_y_ = declare_parameter<double>("rotate.center_y", 0.00);
    rot_total_deg_ = declare_parameter<double>("rotate.total_deg", 90.0);
    rot_step_limit_deg_ = declare_parameter<double>("rotate.step_limit_deg", 3.0);
    rot_with_orientation_ = declare_parameter<bool>("rotate.with_orientation", true);
    rot_max_step_ = declare_parameter<double>("rotate.max_step", 0.004);
    rot_target_speed_ = declare_parameter<double>("rotate.target_speed", 0.010);
    rot_time_scale_ = declare_parameter<double>("rotate.time_scale", 1.2);
    rot_try_without_orientation_ = declare_parameter<bool>("rotate.try_without_orientation", true);
    rot_avoid_collisions_ = declare_parameter<bool>("rotate.avoid_collisions", true);
    rot_fraction_threshold_ = declare_parameter<double>("rotate.fraction_threshold", 0.94);
    rot_settle_sec_ = declare_parameter<double>("rotate.settle_sec", 0.02);

    status_sub_ = create_subscription<std_msgs::msg::String>(
      status_topic_, 10,
      std::bind(&DualArmTransportFullRunner::status_callback, this, std::placeholders::_1));
    state6_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(state6_topic_, 10);

    list_ctrl_client_ = create_client<controller_manager_msgs::srv::ListControllers>(
      "/controller_manager/list_controllers");
    ik_client_ = create_client<moveit_msgs::srv::GetPositionIK>("/compute_ik");

    move_group_client_ = rclcpp_action::create_client<moveit_msgs::action::MoveGroup>(
      this, "/move_action");
    traj_client_ = rclcpp_action::create_client<control_msgs::action::FollowJointTrajectory>(
      this, "/dual_panda_arm_controller/follow_joint_trajectory");
  }

  ~DualArmTransportFullRunner() override
  {
    stop_children();
  }

  bool run()
  {
    RCLCPP_INFO(get_logger(), "C++全流程执行器启动: mode=%s", task_mode_.c_str());

    if (start_sim_) {
      if (!start_sim_process()) {
        return false;
      }
    }

    if (start_native_executor_) {
      if (!start_native_process()) {
        return false;
      }
    }

    if (start_moveit_) {
      if (!start_moveit_process()) {
        return false;
      }
    }

    if (!wait_system_ready()) {
      RCLCPP_ERROR(get_logger(), "系统就绪检查失败。");
      if (auto_shutdown_) {
        stop_children();
      }
      return false;
    }

    const bool ok = execute_transport_task();
    if (auto_shutdown_) {
      stop_children();
    }
    return ok;
  }

private:
  struct ChildProcess
  {
    std::string name;
    pid_t pid;
    std::string log_path;
  };

  bool spawn_process(
    const std::string & name,
    const std::vector<std::string> & args,
    const std::string & log_path)
  {
    if (args.empty()) {
      return false;
    }

    pid_t pid = fork();
    if (pid < 0) {
      RCLCPP_ERROR(get_logger(), "fork 失败: %s", name.c_str());
      return false;
    }

    if (pid == 0) {
      int fd = open(log_path.c_str(), O_CREAT | O_WRONLY | O_TRUNC, 0644);
      if (fd >= 0) {
        dup2(fd, STDOUT_FILENO);
        dup2(fd, STDERR_FILENO);
        close(fd);
      }

      std::vector<char *> cargs;
      cargs.reserve(args.size() + 1);
      for (const auto & s : args) {
        cargs.push_back(const_cast<char *>(s.c_str()));
      }
      cargs.push_back(nullptr);

      execvp(cargs[0], cargs.data());
      _exit(127);
    }

    children_.push_back(ChildProcess{name, pid, log_path});
    RCLCPP_INFO(get_logger(), "已启动进程[%s], pid=%d, log=%s", name.c_str(), pid, log_path.c_str());
    return true;
  }

  void stop_children()
  {
    std::lock_guard<std::mutex> lock(child_mutex_);
    if (children_.empty()) {
      return;
    }

    for (auto it = children_.rbegin(); it != children_.rend(); ++it) {
      if (it->pid <= 0) {
        continue;
      }
      kill(it->pid, SIGINT);
    }

    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (std::chrono::steady_clock::now() < deadline) {
      bool any_alive = false;
      for (auto & p : children_) {
        if (p.pid <= 0) {
          continue;
        }
        int status = 0;
        const pid_t ret = waitpid(p.pid, &status, WNOHANG);
        if (ret == 0) {
          any_alive = true;
        } else {
          p.pid = -1;
        }
      }
      if (!any_alive) {
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    for (auto & p : children_) {
      if (p.pid > 0) {
        kill(p.pid, SIGTERM);
        int status = 0;
        waitpid(p.pid, &status, 0);
        p.pid = -1;
      }
    }
    children_.clear();
  }

  bool start_sim_process()
  {
    const std::vector<std::string> cmd = {
      "ros2", "launch", "franka_bringup", "dual_franka_sim.launch.py",
      "use_rviz:=false", "publish_fallback_target_tf:=true"
    };
    return spawn_process("sim", cmd, sim_log_path_);
  }

  bool start_moveit_process()
  {
    const std::vector<std::string> cmd = {
      "ros2", "launch", "franka_moveit_config", "sim_dual_moveit.launch.py",
      "use_rviz:=false", "launch_mujoco:=false"
    };
    return spawn_process("moveit", cmd, moveit_log_path_);
  }

  bool start_native_process()
  {
    const std::vector<std::string> cmd = {
      "ros2", "run", "dual_arm_transport_native", "dual_arm_transport_native_node",
      "--ros-args", "--params-file",
      "src/multipanda_ros2/dual_arm_transport_native/config/dual_arm_transport_native.yaml"
    };
    return spawn_process("native", cmd, native_log_path_);
  }

  bool wait_for_service_ready(
    const std::string & name,
    const rclcpp::ClientBase::SharedPtr & client,
    double timeout_sec)
  {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout_sec);
    while (std::chrono::steady_clock::now() < deadline) {
      if (client->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_INFO(get_logger(), "服务就绪: %s", name.c_str());
        return true;
      }
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 3000, "等待服务: %s ...", name.c_str());
    }
    RCLCPP_ERROR(get_logger(), "等待服务超时: %s", name.c_str());
    return false;
  }

  template<typename ActionT>
  bool wait_for_action_ready(
    const std::string & name,
    const typename rclcpp_action::Client<ActionT>::SharedPtr & client,
    double timeout_sec)
  {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout_sec);
    while (std::chrono::steady_clock::now() < deadline) {
      if (client->wait_for_action_server(std::chrono::seconds(1))) {
        RCLCPP_INFO(get_logger(), "Action就绪: %s", name.c_str());
        return true;
      }
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 3000, "等待Action: %s ...", name.c_str());
    }
    RCLCPP_ERROR(get_logger(), "等待Action超时: %s", name.c_str());
    return false;
  }

  bool wait_for_topic_link_ready(double timeout_sec)
  {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout_sec);
    while (std::chrono::steady_clock::now() < deadline) {
      const size_t pub_count = count_publishers(status_topic_);
      const size_t sub_count = state6_pub_->get_subscription_count();
      if (pub_count >= 1 && sub_count >= 1) {
        RCLCPP_INFO(
          get_logger(),
          "Topic链路就绪: status publishers=%zu, state6 subscribers=%zu",
          pub_count, sub_count);
        return true;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    RCLCPP_ERROR(get_logger(), "Topic链路超时: status/state6");
    return false;
  }

  bool wait_system_ready()
  {
    const double timeout = std::max(10.0, startup_timeout_sec_);
    if (!wait_for_service_ready("/controller_manager/list_controllers", list_ctrl_client_, timeout)) {
      return false;
    }
    if (!wait_for_service_ready("/compute_ik", ik_client_, timeout)) {
      return false;
    }
    if (!wait_for_action_ready<moveit_msgs::action::MoveGroup>(
        "/move_action", move_group_client_, timeout))
    {
      return false;
    }
    if (!wait_for_action_ready<control_msgs::action::FollowJointTrajectory>(
        "/dual_panda_arm_controller/follow_joint_trajectory", traj_client_, timeout))
    {
      return false;
    }
    if (!wait_for_topic_link_ready(timeout)) {
      return false;
    }
    return true;
  }

  void status_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(status_mutex_);
    last_status_ = msg ? msg->data : "";
  }

  void clear_status()
  {
    std::lock_guard<std::mutex> lock(status_mutex_);
    last_status_.clear();
  }

  std::string get_status()
  {
    std::lock_guard<std::mutex> lock(status_mutex_);
    return last_status_;
  }

  bool wait_native_result(double timeout_sec)
  {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout_sec);
    bool executing_seen = false;
    while (std::chrono::steady_clock::now() < deadline) {
      const std::string status = get_status();
      if (status == "executing") {
        executing_seen = true;
      } else if (status == "succeeded" || status == "failed") {
        RCLCPP_INFO(
          get_logger(),
          "native执行结束: status=%s, executing_seen=%s",
          status.c_str(),
          executing_seen ? "true" : "false");
        return status == "succeeded";
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    RCLCPP_ERROR(get_logger(), "等待native执行结果超时(%.1fs)", timeout_sec);
    return false;
  }

  bool execute_transport_task()
  {
    std_msgs::msg::Float64MultiArray cmd;
    if (task_mode_ == "rotate") {
      cmd.data = {
        1.0,
        rot_center_x_,
        rot_center_y_,
        rot_total_deg_,
        rot_step_limit_deg_,
        rot_with_orientation_ ? 1.0 : 0.0,
        rot_max_step_,
        rot_target_speed_,
        rot_time_scale_,
        rot_try_without_orientation_ ? 1.0 : 0.0,
        rot_avoid_collisions_ ? 1.0 : 0.0,
        rot_fraction_threshold_,
        rot_settle_sec_,
      };
      RCLCPP_INFO(get_logger(), "开始执行C++旋转搬运流程...");
    } else {
      cmd.data = {
        0.0,
        trans_lift_dz_,
        trans_dx_,
        trans_dy_,
        trans_dz_,
        trans_max_step_,
        trans_lift_speed_,
        trans_transport_speed_,
        trans_time_scale_,
        trans_one_shot_segment_,
        trans_retry_segment_,
      };
      RCLCPP_INFO(get_logger(), "开始执行C++平移搬运流程...");
    }

    clear_status();
    state6_pub_->publish(cmd);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    state6_pub_->publish(cmd);

    return wait_native_result(std::max(10.0, native_result_timeout_sec_));
  }

private:
  bool start_sim_;
  bool start_moveit_;
  bool start_native_executor_;
  bool auto_shutdown_;

  std::string sim_log_path_;
  std::string moveit_log_path_;
  std::string native_log_path_;
  std::string state6_topic_;
  std::string status_topic_;

  double startup_timeout_sec_;
  double native_result_timeout_sec_;
  std::string task_mode_;

  double trans_lift_dz_;
  double trans_dx_;
  double trans_dy_;
  double trans_dz_;
  double trans_max_step_;
  double trans_lift_speed_;
  double trans_transport_speed_;
  double trans_time_scale_;
  double trans_one_shot_segment_;
  double trans_retry_segment_;

  double rot_center_x_;
  double rot_center_y_;
  double rot_total_deg_;
  double rot_step_limit_deg_;
  bool rot_with_orientation_;
  double rot_max_step_;
  double rot_target_speed_;
  double rot_time_scale_;
  bool rot_try_without_orientation_;
  bool rot_avoid_collisions_;
  double rot_fraction_threshold_;
  double rot_settle_sec_;

  std::mutex status_mutex_;
  std::string last_status_;

  std::mutex child_mutex_;
  std::vector<ChildProcess> children_;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr state6_pub_;
  rclcpp::Client<controller_manager_msgs::srv::ListControllers>::SharedPtr list_ctrl_client_;
  rclcpp::Client<moveit_msgs::srv::GetPositionIK>::SharedPtr ik_client_;
  rclcpp_action::Client<moveit_msgs::action::MoveGroup>::SharedPtr move_group_client_;
  rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr traj_client_;
};

}  // namespace dual_arm_transport_native

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<dual_arm_transport_native::DualArmTransportFullRunner>();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  std::thread spin_thread([&executor]() { executor.spin(); });
  bool ok = false;
  try {
    ok = node->run();
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node->get_logger(), "运行异常: %s", e.what());
    ok = false;
  } catch (...) {
    RCLCPP_ERROR(node->get_logger(), "运行异常: unknown");
    ok = false;
  }

  executor.cancel();
  if (spin_thread.joinable()) {
    spin_thread.join();
  }
  rclcpp::shutdown();
  return ok ? 0 : 1;
}
