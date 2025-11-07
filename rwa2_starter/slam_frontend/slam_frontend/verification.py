#!/usr/bin/env python3
"""Assignment verification script for RWA2 SLAM Frontend"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from slam_interfaces.msg import Keyframe
import numpy as np


class VerificationNode(Node):
    def __init__(self):
        super().__init__("verification_node")

        self.keyframe_count = 0
        self.last_time = self.get_clock().now()
        self.processing_times = []
        self.path_points = []
        self.start_time = self.get_clock().now()

        # Subscribe to outputs
        self.keyframe_sub = self.create_subscription(
            Keyframe, "/slam/new_keyframe", self.keyframe_callback, 10
        )

        self.path_sub = self.create_subscription(Path, "/path", self.path_callback, 10)

        self.odom_sub = self.create_subscription(
            Odometry, "/odom", self.odom_callback, 10
        )

        # Timer for periodic summary
        self.timer = self.create_timer(10.0, self.print_summary)

        self.get_logger().info(
            "Verification node started - monitoring SLAM performance..."
        )

    def keyframe_callback(self, msg):
        self.keyframe_count += 1
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        if dt > 0 and dt < 1.0:  # Ignore large gaps
            self.processing_times.append(1.0 / dt)
        self.last_time = current_time
        
    def odom_callback(self, msg):
        # This tracks actual processing rate
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        if dt > 0 and dt < 1.0:
            self.processing_times.append(1.0/dt)
        self.last_time = current_time

    def path_callback(self, msg):
        if len(msg.poses) > 0:
            last_pose = msg.poses[-1].pose
            self.path_points.append(
                [last_pose.position.x, last_pose.position.y, last_pose.position.z]
            )

    def print_summary(self):
        elapsed_time = (self.get_clock().now() - self.start_time).nanoseconds / 1e9

        if len(self.processing_times) > 0:
            avg_hz = np.mean(self.processing_times[-100:])  # Last 100 samples

            self.get_logger().info(f"\n{'=' * 50}")
            self.get_logger().info(f"VERIFICATION SUMMARY (t={elapsed_time:.1f}s)")
            self.get_logger().info(f"{'=' * 50}")
            self.get_logger().info(f"Keyframes generated: {self.keyframe_count}")
            self.get_logger().info(f"Average processing rate: {avg_hz:.2f} Hz")

            if len(self.path_points) > 10:
                points = np.array(self.path_points)

                # Calculate metrics
                total_dist = np.sum(np.linalg.norm(points[1:] - points[:-1], axis=1))
                z_range = points[:, 2].max() - points[:, 2].min()
                current_pos = points[-1]

                # Check for jumps
                if len(points) > 1:
                    diffs = np.linalg.norm(points[1:] - points[:-1], axis=1)
                    max_jump = np.max(diffs)
                else:
                    max_jump = 0

                self.get_logger().info(f"Total distance traveled: {total_dist:.1f}m")
                self.get_logger().info(f"Z-axis variation: {z_range:.2f}m")
                self.get_logger().info(f"Max jump between poses: {max_jump:.2f}m")
                self.get_logger().info(
                    f"Current position: [{current_pos[0]:.1f}, "
                    f"{current_pos[1]:.1f}, {current_pos[2]:.1f}]"
                )

                # Performance checks
                self.get_logger().info(f"\n{'=' * 50}")
                self.get_logger().info("PERFORMANCE CHECKS:")
                self.get_logger().info(f"{'=' * 50}")

                all_passed = True

                # Check keyframe count (expected 200-400 for full sequence)
                if elapsed_time > 60:  # After 1 minute
                    keyframe_rate = (
                        self.keyframe_count / elapsed_time * 250
                    )  # Estimate for full sequence
                    if keyframe_rate < 100:
                        self.get_logger().warn(
                            f"❌ Too few keyframes (projected: {keyframe_rate:.0f})"
                        )
                        all_passed = False
                    elif keyframe_rate > 600:
                        self.get_logger().warn(
                            f"❌ Too many keyframes (projected: {keyframe_rate:.0f})"
                        )
                        all_passed = False
                    else:
                        self.get_logger().info("✅ Keyframe generation rate OK")

                # Check processing speed
                if avg_hz < 5.0:
                    self.get_logger().warn(
                        f"❌ Processing too slow ({avg_hz:.1f} Hz < 5 Hz)"
                    )
                    all_passed = False
                else:
                    self.get_logger().info(f"✅ Processing speed OK ({avg_hz:.1f} Hz)")

                # Check Z drift
                if z_range > 4.0:
                    self.get_logger().warn(
                        f"❌ Excessive Z-drift ({z_range:.1f}m > 4m)"
                    )
                    all_passed = False
                else:
                    self.get_logger().info(f"✅ Z-drift within limits ({z_range:.1f}m)")

                # Check for large jumps
                if max_jump > 5.0:
                    self.get_logger().warn(
                        f"❌ Large jump detected ({max_jump:.1f}m > 5m)"
                    )
                    all_passed = False
                else:
                    self.get_logger().info(
                        f"✅ Path continuity OK (max jump: {max_jump:.1f}m)"
                    )

                # Check minimum distance
                if elapsed_time > 120 and total_dist < 50:  # After 2 minutes
                    self.get_logger().warn(
                        f"❌ Vehicle not moving enough ({total_dist:.1f}m)"
                    )
                    all_passed = False
                elif total_dist > 10:
                    self.get_logger().info("✅ Vehicle is moving")

                if all_passed:
                    self.get_logger().info("\n🎉 ALL CHECKS PASSING! 🎉")
                else:
                    self.get_logger().info("\n⚠️  Some checks need attention")

            self.get_logger().info(f"{'=' * 50}\n")


def main(args=None):
    rclpy.init(args=args)

    verification_node = VerificationNode()

    try:
        rclpy.spin(verification_node)
    except KeyboardInterrupt:
        # Print final summary
        verification_node.get_logger().info("\n\nFINAL SUMMARY:")
        verification_node.print_summary()
    finally:
        verification_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
