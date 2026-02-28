#!/usr/bin/env python3
#以 50Hz 的高频监控并记录 TF2 坐标变换池 (TF buffer) 的同步延迟和误差，常用于排查双臂动作不同步的问题。
import rclpy
import csv
import matplotlib.pyplot as plt
from rclpy.node import Node
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class SyncErrorLogger(Node):
    def __init__(self):
        super().__init__('sync_error_logger')
        
        # Initialize tf2_ros buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Create a timer at 50Hz (0.02 seconds)
        self.timer = self.create_timer(0.02, self.timer_callback)
        self.get_logger().info('SyncErrorLogger node initialized at 50Hz')

        # Data logging initialization
        self.data_log = []
        self.start_time = self.get_clock().now()

    def timer_callback(self):
        try:
            # Try to lookup transform for panda_1_link8
            t1 = self.tf_buffer.lookup_transform(
                'world',
                'panda_1_link8',
                rclpy.time.Time())
                
            # Try to lookup transform for panda_2_link8
            t2 = self.tf_buffer.lookup_transform(
                'world',
                'panda_2_link8',
                rclpy.time.Time())

            # Get Z coordinates
            z1 = t1.transform.translation.z
            z2 = t2.transform.translation.z
            
            # Calculate error
            error = abs(z1 - z2)
            
            # Calculate elapsed time
            current_time = self.get_clock().now()
            elapsed_time = (current_time - self.start_time).nanoseconds / 1e9
            
            # Log data
            self.data_log.append((elapsed_time, z1, z2, error))
            self.get_logger().info(f'Current error: {error:.4f}')
            
        except TransformException as ex:
            self.get_logger().warn(f'Could not transform: {ex}')

    def save_data_and_plot(self):
        if not self.data_log:
            self.get_logger().warn('No data to save or plot.')
            return

        # --- Save CSV ---
        csv_filename = 'sync_error_data.csv'
        try:
            with open(csv_filename, mode='w', newline='') as file:
                writer = csv.writer(file)
                writer.writerow(['Time', 'Z1', 'Z2', 'Error'])
                writer.writerows(self.data_log)
            self.get_logger().info(f'Data saved to {csv_filename}')
        except Exception as e:
            self.get_logger().error(f'Failed to save CSV: {e}')

        # --- Plot Data ---
        try:
            times = [row[0] for row in self.data_log]
            z1s = [row[1] for row in self.data_log]
            z2s = [row[2] for row in self.data_log]
            errors = [row[3] for row in self.data_log]

            fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True, figsize=(10, 8))

            # Subplot 1: Z1 and Z2
            ax1.plot(times, z1s, label='Z1 (Blue)', color='blue')
            ax1.plot(times, z2s, label='Z2 (Red)', color='red')
            ax1.set_ylabel('Height (m)')
            ax1.set_title('Z Position of End Effectors')
            ax1.legend()
            ax1.grid(True)

            # Subplot 2: Error
            ax2.plot(times, errors, label='Error (Green)', color='green')
            # Add reference line y=0.005
            ax2.axhline(y=0.005, color='red', linestyle='--', label='Reference (5mm)')
            ax2.set_xlabel('Time (s)')
            ax2.set_ylabel('Error (m)')
            ax2.set_title('Synchronization Error')
            ax2.legend()
            ax2.grid(True)

            plot_filename = 'sync_error_plot.png'
            plt.savefig(plot_filename)
            self.get_logger().info(f'Plot saved to {plot_filename}')
            plt.show()
        except Exception as e:
            self.get_logger().error(f'Failed to plot data: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = SyncErrorLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # User pressed Ctrl+C
        pass
    except Exception as e:
        node.get_logger().error(f'Error during spin: {e}')
    finally:
        # Save data and plot before destroying node
        node.get_logger().info('Shutting down, saving data...')
        node.save_data_and_plot()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
