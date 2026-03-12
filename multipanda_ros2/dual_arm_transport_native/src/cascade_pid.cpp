#include "dual_arm_transport_native/cascade_pid.hpp"

#include <algorithm>
#include <cmath>

namespace dual_arm_transport_native
{

namespace
{

double clamp_scalar(double value, double min_v, double max_v)
{
  if (value < min_v) {
    return min_v;
  }
  if (value > max_v) {
    return max_v;
  }
  return value;
}

}  // namespace

void cascade_pid_reset_joint(CascadePidJointState * state)
{
  if (state == nullptr) {
    return;
  }
  state->outer_integral = 0.0;
  state->inner_integral = 0.0;
  state->prev_pos_error = 0.0;
  state->prev_vel_error = 0.0;
  state->prev_cmd = 0.0;
}

void cascade_pid_reset_all(CascadePidJointState * states, int joint_count)
{
  if (states == nullptr || joint_count <= 0) {
    return;
  }
  for (int i = 0; i < joint_count; ++i) {
    cascade_pid_reset_joint(&states[i]);
  }
}

double cascade_pid_step_joint(
  const CascadePidJointGains * gains,
  CascadePidJointState * state,
  double q_ref,
  double dq_ref,
  double q_meas,
  double dq_meas,
  double dt,
  double outer_i_clamp,
  double inner_i_clamp,
  double outer_v_limit,
  double cmd_vel_limit,
  double cmd_acc_limit)
{
  if (gains == nullptr || state == nullptr) {
    return 0.0;
  }

  dt = clamp_scalar(dt, 1e-4, 0.2);

  const double pos_error = q_ref - q_meas;
  state->outer_integral = clamp_scalar(
    state->outer_integral + pos_error * dt,
    -std::fabs(outer_i_clamp),
    std::fabs(outer_i_clamp));
  const double d_pos_error = (pos_error - state->prev_pos_error) / dt;

  double vel_ref = dq_ref +
    gains->outer_kp * pos_error +
    gains->outer_ki * state->outer_integral +
    gains->outer_kd * d_pos_error;
  vel_ref = clamp_scalar(vel_ref, -std::fabs(outer_v_limit), std::fabs(outer_v_limit));

  const double vel_error = vel_ref - dq_meas;
  state->inner_integral = clamp_scalar(
    state->inner_integral + vel_error * dt,
    -std::fabs(inner_i_clamp),
    std::fabs(inner_i_clamp));
  const double d_vel_error = (vel_error - state->prev_vel_error) / dt;

  double cmd = vel_ref +
    gains->inner_kp * vel_error +
    gains->inner_ki * state->inner_integral +
    gains->inner_kd * d_vel_error;

  const double dv_max = std::fabs(cmd_acc_limit) * dt;
  cmd = state->prev_cmd + clamp_scalar(cmd - state->prev_cmd, -dv_max, dv_max);
  cmd = clamp_scalar(cmd, -std::fabs(cmd_vel_limit), std::fabs(cmd_vel_limit));

  state->prev_pos_error = pos_error;
  state->prev_vel_error = vel_error;
  state->prev_cmd = cmd;

  return cmd;
}

void cascade_pid_step_all(
  const CascadePidJointGains * gains,
  CascadePidJointState * states,
  int joint_count,
  const double * q_ref,
  const double * dq_ref,
  const double * q_meas,
  const double * dq_meas,
  double dt,
  double outer_i_clamp,
  double inner_i_clamp,
  double outer_v_limit,
  double cmd_vel_limit,
  double cmd_acc_limit,
  double * out_cmd_vel)
{
  if (
    gains == nullptr || states == nullptr || joint_count <= 0 ||
    q_ref == nullptr || dq_ref == nullptr || q_meas == nullptr || dq_meas == nullptr ||
    out_cmd_vel == nullptr)
  {
    return;
  }

  for (int i = 0; i < joint_count; ++i) {
    out_cmd_vel[i] = cascade_pid_step_joint(
      &gains[i],
      &states[i],
      q_ref[i],
      dq_ref[i],
      q_meas[i],
      dq_meas[i],
      dt,
      outer_i_clamp,
      inner_i_clamp,
      outer_v_limit,
      cmd_vel_limit,
      cmd_acc_limit);
  }
}

}  // namespace dual_arm_transport_native
