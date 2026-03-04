#!/usr/bin/env python3

#夹爪逻辑

import asyncio

from action_msgs.msg import GoalStatus
from control_msgs.action import GripperCommand


class GripperOpsMixin:
    def _gripper_warn_once(self, key: str, message: str):
        warned = getattr(self, '_gripper_warned_keys', set())
        if key in warned:
            return
        warned.add(key)
        self._gripper_warned_keys = warned
        self.get_logger().warn(message)

    def _finger_opening(self, side: str):
        joint_name = 'mj_left_finger_joint1' if side == 'left' else 'mj_right_finger_joint1'
        return self.current_joint_state.get(joint_name, None)

    def _normalize_gripper_position(self, position):
        """将不同来源的夹爪开度统一为“单指行程(0~0.04m)”."""
        if position is None:
            return None
        try:
            value = float(position)
        except (TypeError, ValueError):
            return None
        if value < 0.0:
            return None
        return self._clamp_gripper_position(value)

    def get_gripper_opening_estimate(self, side: str):
        """优先用 /joint_states；若该通道不可靠则回退 action result 缓存."""
        opening = self._normalize_gripper_position(self._finger_opening(side))
        if opening is not None and opening > 1e-6:
            return opening

        cache = getattr(self, 'last_gripper_result', {}).get(side, {})
        cached_opening = self._normalize_gripper_position(cache.get('position'))
        return cached_opening

    def _clamp_gripper_position(self, position: float) -> float:
        return max(0.0, min(float(position), float(self.gripper_open_position)))

    def _cache_gripper_result(self, side: str, status: int, normalized_pos, raw_position,
                              reached_goal=None, stalled=None, source: str = 'gripper_action'):
        if not hasattr(self, 'last_gripper_result'):
            self.last_gripper_result = {}
        self.last_gripper_result[side] = {
            'status': int(status),
            'position': normalized_pos,
            'raw_position': raw_position,
            'reached_goal': reached_goal,
            'stalled': stalled,
            'source': source,
        }

    async def _control_gripper_via_move_async(self, side: str, position: float,
                                              wait_for_result: bool = True) -> bool:
        """
        兜底通道: franka_msgs/Move（width 为总开度 0~0.08m）。
        """
        if not hasattr(self, '_gripper_move_clients_ready'):
            self._gripper_move_clients_ready = False
            self._franka_move_action_type = None
            self.left_gripper_move_client = None
            self.right_gripper_move_client = None
            try:
                from rclpy.action import ActionClient
                from franka_msgs.action import Move as FrankaMove
                cb_group = getattr(self, 'cb_group_action', None)
                left_name = '/mj_left_gripper_sim_node/move'
                right_name = '/mj_right_gripper_sim_node/move'
                if cb_group is None:
                    self.left_gripper_move_client = ActionClient(self, FrankaMove, left_name)
                    self.right_gripper_move_client = ActionClient(self, FrankaMove, right_name)
                else:
                    self.left_gripper_move_client = ActionClient(
                        self, FrankaMove, left_name, callback_group=cb_group)
                    self.right_gripper_move_client = ActionClient(
                        self, FrankaMove, right_name, callback_group=cb_group)
                self._franka_move_action_type = FrankaMove
                self._gripper_move_clients_ready = True
            except Exception as e:
                self._gripper_warn_once(
                    'move_import_failed',
                    f"franka_msgs/Move 兜底通道不可用({e})，仅使用 GripperCommand")
                return False

        client = self.left_gripper_move_client if side == 'left' else self.right_gripper_move_client
        if client is None:
            return False
        if not client.server_is_ready() and not client.wait_for_server(timeout_sec=0.2):
            return False

        goal = self._franka_move_action_type.Goal()
        goal.width = max(0.0, min(0.08, float(position) * 2.0))
        goal.speed = float(getattr(self, 'gripper_move_speed', 0.08))
        try:
            goal_handle = await asyncio.wait_for(
                client.send_goal_async(goal),
                timeout=float(getattr(self, 'gripper_goal_response_timeout_sec', 3.0)),
            )
            if not goal_handle.accepted:
                return False
            if not wait_for_result:
                return True

            result = await asyncio.wait_for(
                goal_handle.get_result_async(),
                timeout=float(getattr(self, 'gripper_result_timeout_sec', 6.0)),
            )
            status = int(getattr(result, 'status', GoalStatus.STATUS_UNKNOWN))
            result_msg = getattr(result, 'result', None)
            move_success = bool(getattr(result_msg, 'success', False)) if result_msg is not None else False
            cur_opening = self.get_gripper_opening_estimate(side)
            self._cache_gripper_result(
                side=side,
                status=status,
                normalized_pos=cur_opening,
                raw_position=None if cur_opening is None else cur_opening * 2.0,
                reached_goal=move_success,
                stalled=None,
                source='move_action',
            )
            return move_success or status in (GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_ABORTED)
        except Exception:
            return False

    async def _sync_grasp_via_trajectory_fallback(self, width: float, wait_for_result: bool = True) -> bool:
        """
        最后兜底: 尝试通过 joint_trajectory_controller 直接给两指关节下发目标。
        若控制器不接受手指关节，该分支会快速失败并返回 False。
        """
        client = getattr(self, 'dual_controller_client', None)
        if client is None:
            return False
        if not client.server_is_ready() and not client.wait_for_server(timeout_sec=0.2):
            return False

        try:
            from builtin_interfaces.msg import Duration
            from control_msgs.action import FollowJointTrajectory
            from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
        except Exception:
            return False

        goal = FollowJointTrajectory.Goal()
        traj = JointTrajectory()
        traj.joint_names = ['mj_left_finger_joint1', 'mj_right_finger_joint1']
        pt = JointTrajectoryPoint()
        pt.positions = [float(width), float(width)]
        pt.velocities = [0.0, 0.0]
        pt.time_from_start = Duration(sec=0, nanosec=600000000)
        traj.points = [pt]
        goal.trajectory = traj

        try:
            goal_handle = await asyncio.wait_for(client.send_goal_async(goal), timeout=2.0)
            if not goal_handle.accepted:
                return False
            if not wait_for_result:
                return True
            result = await asyncio.wait_for(goal_handle.get_result_async(), timeout=4.0)
            return int(getattr(result, 'status', -1)) in (
                GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_ABORTED)
        except Exception:
            return False

    async def _gripper_hold_loop(self):
        interval = max(0.15, float(getattr(self, 'gripper_hold_interval_sec', 0.35)))
        while bool(getattr(self, '_gripper_hold_active', False)):
            width = float(getattr(self, '_gripper_hold_width', self.gripper_closed_position))
            effort = float(getattr(self, '_gripper_hold_effort', self.gripper_effort_close))
            try:
                await self.sync_grasp(width, effort, wait_for_result=False)
            except Exception as e:
                self._gripper_warn_once('hold_loop_error', f"夹爪保压循环异常: {e}")
            await asyncio.sleep(interval)

    async def start_gripper_hold(self, width=None, effort=None):
        """启动后台保压闭爪，防止搬运阶段松脱。"""
        if width is None:
            width = self.gripper_closed_position
        if effort is None:
            effort = self.gripper_effort_close
        await self.stop_gripper_hold("重启保压")
        self._gripper_hold_width = self._clamp_gripper_position(width)
        self._gripper_hold_effort = float(effort)
        self._gripper_hold_active = True
        self._gripper_hold_task = asyncio.create_task(self._gripper_hold_loop())
        self.get_logger().info(
            f"已启动夹爪保压循环: width={self._gripper_hold_width:.3f}, effort={self._gripper_hold_effort:.1f}")

    async def stop_gripper_hold(self, reason: str = ""):
        task = getattr(self, '_gripper_hold_task', None)
        self._gripper_hold_active = False
        if task is None:
            return
        if not task.done():
            try:
                await asyncio.wait_for(task, timeout=1.0)
            except asyncio.TimeoutError:
                task.cancel()
            except Exception:
                pass
        self._gripper_hold_task = None
        if reason:
            self.get_logger().info(f"夹爪保压循环已停止: {reason}")

    async def ensure_grippers_open(self, target=None, retries: int = 3) -> bool:
        """确保双爪张开到可用开度；避免“动作返回但实际未打开”."""
        if target is None:
            target = float(self.gripper_open_position)
        target = self._clamp_gripper_position(target)

        for attempt in range(1, retries + 1):
            # 避免贴边上限导致 action server 报 ABORTED，预留 1mm
            cmd_pos = max(0.0, target - 0.001 * (attempt - 1))
            await self.sync_grasp(cmd_pos, self.gripper_effort_open, wait_for_result=True)
            await asyncio.sleep(0.2)
            left = self.get_gripper_opening_estimate('left')
            right = self.get_gripper_opening_estimate('right')
            if left is not None and right is not None:
                self.get_logger().info(
                    f"[开爪校验] attempt={attempt}/{retries}, target={cmd_pos:.3f}, "
                    f"left={left:.4f}, right={right:.4f}")
                if left >= cmd_pos * 0.65 and right >= cmd_pos * 0.65:
                    return True
            else:
                self.get_logger().warn("[开爪校验] 尚未获取到夹爪关节状态，继续重试")
        self.get_logger().warn("开爪校验未达标，继续执行中期流程")
        return False

    async def close_grippers_on_exit(self, tag: str = "任务退出收尾") -> bool:
        """程序退出前的统一闭爪动作。"""
        target = self._clamp_gripper_position(self.gripper_closed_position)
        effort = float(self.gripper_effort_close)
        try:
            await self.stop_gripper_hold("程序退出")
            self.get_logger().info(f"[{tag}] 执行退出闭爪: pos={target:.3f}m, effort={effort:.1f}N")
            await self.sync_grasp(target, effort, wait_for_result=True)
            await asyncio.sleep(0.2)
            left = self.get_gripper_opening_estimate('left')
            right = self.get_gripper_opening_estimate('right')
            self.get_logger().info(
                f"[{tag}] 闭爪后指关节: left={left}, right={right}")
            return True
        except Exception as e:
            self.get_logger().warn(f"[{tag}] 退出闭爪失败: {e}")
            return False

    async def control_gripper_async(self, side: str, position: float, max_effort: float,
                                     wait_for_result: bool = True):
        client = self.left_gripper_client if side == 'left' else self.right_gripper_client
        goal = GripperCommand.Goal()
        requested_position = float(position)
        clamped_position = self._clamp_gripper_position(requested_position)
        if abs(clamped_position - requested_position) > 1e-6:
            self.get_logger().warn(
                f"{side}夹爪目标超范围: req={requested_position:.3f}m -> clamp={clamped_position:.3f}m")
        goal.command.position = clamped_position
        goal.command.max_effort = max_effort
        
        open_threshold = 0.8 * float(self.gripper_open_position)
        action_desc = "闭合" if clamped_position < open_threshold else "张开"
        self.get_logger().info(f"夹爪{action_desc} {side}: pos={clamped_position:.3f}m, effort={max_effort:.0f}N")
        command_ok = False
        try:
            goal_handle = await asyncio.wait_for(
                client.send_goal_async(goal),
                timeout=float(getattr(self, 'gripper_goal_response_timeout_sec', 3.0)))
            if not goal_handle.accepted:
                self.get_logger().warn(f"{side}夹爪目标被拒绝，准备尝试兜底通道")
                command_ok = False
            elif not wait_for_result:
                command_ok = True
            else:
                try:
                    result = await asyncio.wait_for(
                        goal_handle.get_result_async(),
                        timeout=float(getattr(self, 'gripper_result_timeout_sec', 6.0)))
                    result_msg = getattr(result, 'result', None)
                    raw_position = None if result_msg is None else getattr(result_msg, 'position', None)
                    normalized_pos = None
                    if raw_position is not None:
                        # franka_gripper Result.position 是总开度(两指之和)，此处转成单指行程
                        normalized_pos = self._normalize_gripper_position(float(raw_position) * 0.5)
                    self._cache_gripper_result(
                        side=side,
                        status=int(result.status),
                        normalized_pos=normalized_pos,
                        raw_position=raw_position,
                        reached_goal=None if result_msg is None else getattr(result_msg, 'reached_goal', None),
                        stalled=None if result_msg is None else getattr(result_msg, 'stalled', None),
                        source='gripper_action',
                    )
                    if result.status == GoalStatus.STATUS_SUCCEEDED:
                        self.get_logger().info(
                            f"{side}夹爪{action_desc}完成(状态={result.status}, "
                            f"opening={normalized_pos})")
                        command_ok = True
                    elif result.status == GoalStatus.STATUS_ABORTED:
                        cur_opening = self.get_gripper_opening_estimate(side)
                        self.get_logger().warn(
                            f"{side}夹爪{action_desc}返回 ABORTED(6)，当前指关节={cur_opening}")
                        # 在仿真中 ABORTED 也常伴随实际运动，先视为软成功
                        command_ok = True
                    else:
                        self.get_logger().warn(
                            f"{side}夹爪{action_desc}完成但状态异常={result.status}, "
                            f"opening={normalized_pos}")
                        command_ok = False
                except asyncio.TimeoutError:
                    self.get_logger().warn(f"{side}夹爪等待结果超时，准备尝试兜底通道")
                    command_ok = False
                except Exception as e:
                    self.get_logger().warn(f"{side}夹爪结果处理异常({e})，准备尝试兜底通道")
                    command_ok = False

        except asyncio.TimeoutError:
            self.get_logger().warn(f"{side}夹爪发送目标超时，准备尝试兜底通道")
            command_ok = False
        except Exception as e:
            self.get_logger().warn(f"{side}夹爪通讯异常({e})，准备尝试兜底通道")
            command_ok = False

        if command_ok:
            return True

        move_ok = await self._control_gripper_via_move_async(
            side=side,
            position=clamped_position,
            wait_for_result=wait_for_result,
        )
        if move_ok:
            self.get_logger().info(f"{side}夹爪通过 Move 兜底通道执行成功")
            return True
        return False

    async def sync_grasp(self, width: float, effort: float, wait_for_result: bool = True):
        """同步控制双爪"""
        width = self._clamp_gripper_position(width)
        left_ok, right_ok = await asyncio.gather(
            self.control_gripper_async('left', width, effort, wait_for_result),
            self.control_gripper_async('right', width, effort, wait_for_result)
        )
        if left_ok and right_ok:
            return True

        traj_ok = await self._sync_grasp_via_trajectory_fallback(width, wait_for_result=wait_for_result)
        if traj_ok:
            self.get_logger().warn(
                "GripperCommand/Move 均失败，已通过 trajectory 兜底通道执行夹爪目标")
            return True
        return False
