#ifndef DUAL_ARM_TRANSPORT_NATIVE__CASCADE_PID_HPP_
#define DUAL_ARM_TRANSPORT_NATIVE__CASCADE_PID_HPP_

namespace dual_arm_transport_native
{

struct CascadePidJointGains
{
  double outer_kp;
  double outer_ki;
  double outer_kd;
  double inner_kp;
  double inner_ki;
  double inner_kd;
};

struct CascadePidJointState
{
  double outer_integral;
  double inner_integral;
  double prev_pos_error;
  double prev_vel_error;
  double prev_cmd;
};

void cascade_pid_reset_joint(CascadePidJointState * state);

void cascade_pid_reset_all(CascadePidJointState * states, int joint_count);

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
  double cmd_acc_limit);

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
  double * out_cmd_vel);

}  // namespace dual_arm_transport_native

#endif  // DUAL_ARM_TRANSPORT_NATIVE__CASCADE_PID_HPP_
