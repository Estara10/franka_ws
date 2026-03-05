#!/usr/bin/env python3
"""通信与同步量化记录器（中期答辩指标版）。

默认记录:
- /joint_states
- /mj_left_arm/force_torque_sensor
- /mj_right_arm/force_torque_sensor
- TF(mj_left_link8, mj_right_link8) 的同步误差

输出:
- comm_sync_metrics.csv
- comm_sync_summary.txt
"""

import csv
import math
from pathlib import Path

import rclpy
from geometry_msgs.msg import WrenchStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


def _stamp_to_sec(stamp_msg):
    sec = int(getattr(stamp_msg, 'sec', 0))
    nsec = int(getattr(stamp_msg, 'nanosec', 0))
    if sec == 0 and nsec == 0:
        return None
    return float(sec) + float(nsec) * 1e-9


def _p95(values):
    if not values:
        return None
    sorted_vals = sorted(values)
    idx = max(0, min(len(sorted_vals) - 1, int(math.ceil(0.95 * len(sorted_vals))) - 1))
    return sorted_vals[idx]


def _stats(values):
    vals = [float(v) for v in values if v is not None]
    if not vals:
        return {'count': 0, 'mean': None, 'p95': None, 'max': None}
    return {
        'count': len(vals),
        'mean': sum(vals) / len(vals),
        'p95': _p95(vals),
        'max': max(vals),
    }


class SyncErrorLogger(Node):
    def __init__(self):
        super().__init__('sync_error_logger')

        self.declare_parameter('planning_frame', 'base_link')
        self.declare_parameter('left_ee_link', 'mj_left_link8')
        self.declare_parameter('right_ee_link', 'mj_right_link8')
        self.declare_parameter('expected_abs_y_distance', 0.20)
        self.declare_parameter('timer_period_sec', 0.02)  # 50Hz

        self.planning_frame = str(self.get_parameter('planning_frame').value)
        self.left_ee_link = str(self.get_parameter('left_ee_link').value)
        self.right_ee_link = str(self.get_parameter('right_ee_link').value)
        self.expected_abs_y_distance = float(self.get_parameter('expected_abs_y_distance').value)
        timer_period = float(self.get_parameter('timer_period_sec').value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.latest = {
            'joint_states': {'age_ms': None, 'dt_ms': None},
            'left_wrench': {'age_ms': None, 'dt_ms': None},
            'right_wrench': {'age_ms': None, 'dt_ms': None},
        }
        self.last_recv_sec = {
            'joint_states': None,
            'left_wrench': None,
            'right_wrench': None,
        }

        self.data_log = []
        self.start_time_sec = self._now_sec()
        self.last_progress_print_sec = self.start_time_sec

        self.create_subscription(JointState, '/joint_states', self._joint_cb, 50)
        self.create_subscription(
            WrenchStamped, '/mj_left_arm/force_torque_sensor', self._left_wrench_cb, 50
        )
        self.create_subscription(
            WrenchStamped, '/mj_right_arm/force_torque_sensor', self._right_wrench_cb, 50
        )
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info(
            f'SyncErrorLogger started ({1.0 / timer_period:.1f}Hz): frame={self.planning_frame}, '
            f'left={self.left_ee_link}, right={self.right_ee_link}'
        )

    def _now_sec(self):
        return float(self.get_clock().now().nanoseconds) * 1e-9

    def _update_topic_metrics(self, key: str, msg):
        now_sec = self._now_sec()
        stamp_sec = _stamp_to_sec(msg.header.stamp) if hasattr(msg, 'header') else None
        age_ms = None if stamp_sec is None else max(0.0, (now_sec - stamp_sec) * 1000.0)

        prev_recv = self.last_recv_sec[key]
        dt_ms = None if prev_recv is None else max(0.0, (now_sec - prev_recv) * 1000.0)
        self.last_recv_sec[key] = now_sec

        self.latest[key]['age_ms'] = age_ms
        self.latest[key]['dt_ms'] = dt_ms

    def _joint_cb(self, msg: JointState):
        self._update_topic_metrics('joint_states', msg)

    def _left_wrench_cb(self, msg: WrenchStamped):
        self._update_topic_metrics('left_wrench', msg)

    def _right_wrench_cb(self, msg: WrenchStamped):
        self._update_topic_metrics('right_wrench', msg)

    def timer_callback(self):
        now_sec = self._now_sec()
        elapsed_sec = now_sec - self.start_time_sec

        z_gap_mm = None
        y_dist_error_mm = None
        try:
            t_left = self.tf_buffer.lookup_transform(
                self.planning_frame, self.left_ee_link, rclpy.time.Time()
            )
            t_right = self.tf_buffer.lookup_transform(
                self.planning_frame, self.right_ee_link, rclpy.time.Time()
            )
            y_left = float(t_left.transform.translation.y)
            y_right = float(t_right.transform.translation.y)
            z_left = float(t_left.transform.translation.z)
            z_right = float(t_right.transform.translation.z)

            z_gap_mm = abs(z_left - z_right) * 1000.0
            abs_y_dist = abs(y_left - y_right)
            y_dist_error_mm = abs(abs_y_dist - self.expected_abs_y_distance) * 1000.0
        except TransformException:
            pass

        row = {
            'time_s': elapsed_sec,
            'joint_age_ms': self.latest['joint_states']['age_ms'],
            'joint_dt_ms': self.latest['joint_states']['dt_ms'],
            'left_wrench_age_ms': self.latest['left_wrench']['age_ms'],
            'left_wrench_dt_ms': self.latest['left_wrench']['dt_ms'],
            'right_wrench_age_ms': self.latest['right_wrench']['age_ms'],
            'right_wrench_dt_ms': self.latest['right_wrench']['dt_ms'],
            'tf_z_gap_mm': z_gap_mm,
            'tf_y_dist_error_mm': y_dist_error_mm,
        }
        self.data_log.append(row)

        if now_sec - self.last_progress_print_sec >= 1.0:
            self.last_progress_print_sec = now_sec
            self.get_logger().info(
                f"[sample={len(self.data_log)}] "
                f"joint_age={row['joint_age_ms']}, "
                f"l_wrench_age={row['left_wrench_age_ms']}, "
                f"r_wrench_age={row['right_wrench_age_ms']}, "
                f"tf_z_gap_mm={row['tf_z_gap_mm']}"
            )

    def save_reports(self):
        if not self.data_log:
            self.get_logger().warn('No data collected. Skip saving.')
            return

        csv_path = Path('comm_sync_metrics.csv')
        summary_path = Path('comm_sync_summary.txt')

        fieldnames = [
            'time_s',
            'joint_age_ms',
            'joint_dt_ms',
            'left_wrench_age_ms',
            'left_wrench_dt_ms',
            'right_wrench_age_ms',
            'right_wrench_dt_ms',
            'tf_z_gap_mm',
            'tf_y_dist_error_mm',
        ]
        with csv_path.open('w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(self.data_log)

        summary_metrics = {
            'joint_age_ms': _stats([r['joint_age_ms'] for r in self.data_log]),
            'joint_dt_ms': _stats([r['joint_dt_ms'] for r in self.data_log]),
            'left_wrench_age_ms': _stats([r['left_wrench_age_ms'] for r in self.data_log]),
            'left_wrench_dt_ms': _stats([r['left_wrench_dt_ms'] for r in self.data_log]),
            'right_wrench_age_ms': _stats([r['right_wrench_age_ms'] for r in self.data_log]),
            'right_wrench_dt_ms': _stats([r['right_wrench_dt_ms'] for r in self.data_log]),
            'tf_z_gap_mm': _stats([r['tf_z_gap_mm'] for r in self.data_log]),
            'tf_y_dist_error_mm': _stats([r['tf_y_dist_error_mm'] for r in self.data_log]),
        }

        with summary_path.open('w') as f:
            f.write(f"samples={len(self.data_log)}\n")
            f.write(
                "metric,count,mean,p95,max\n"
            )
            for key, st in summary_metrics.items():
                f.write(
                    f"{key},{st['count']},{st['mean']},{st['p95']},{st['max']}\n"
                )

        self.get_logger().info(f'Data saved: {csv_path.resolve()}')
        self.get_logger().info(f'Summary saved: {summary_path.resolve()}')

        for metric_name, st in summary_metrics.items():
            self.get_logger().info(
                f"{metric_name}: count={st['count']} mean={st['mean']} p95={st['p95']} max={st['max']}"
            )


def main(args=None):
    rclpy.init(args=args)
    node = SyncErrorLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('Shutting down, exporting reports...')
        node.save_reports()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
