#!/usr/bin/python3
import math
import threading
import rclpy
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist


class Commander(Node):

    def __init__(self):
        super().__init__('commander')
        timer_period = 0.02
        self.wheel_separation = 0.936
        self.wheel_base = 1.09
        self.wheel_radius = 0.2378
        self.wheel_steering_y_offset = 0.075
        self.steering_track = self.wheel_separation - 2 * self.wheel_steering_y_offset

        self.pos = np.array([0, 0, 0, 0], float)
        self.vel = np.array([0, 0, 0, 0], float)  # left_front, right_front, left_rear, right_rear

        self.pub_pos = self.create_publisher(Float64MultiArray, '/forward_position_controller/commands', 10)
        self.pub_vel = self.create_publisher(Float64MultiArray, '/forward_velocity_controller/commands', 10)

        # Subscribe to /cmd_vel
        self.sub_cmd_vel = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.vel_msg = Twist()  # Store the latest velocity message

    def cmd_vel_callback(self, msg):
        """Callback for the /cmd_vel topic to update the robot's velocity and position commands."""
        self.vel_msg = msg

    def timer_callback(self):
        """Calculates the wheel velocities and steering angles based on the received /cmd_vel data."""
        vel_msg = self.vel_msg

        if vel_msg.linear.x != 0 or vel_msg.angular.z != 0:
            # Calculate steering angles and velocities based on the twist message
            vel_steering_offset = vel_msg.angular.z * self.wheel_steering_y_offset
            sign = -np.sign(vel_msg.linear.x)

            self.vel[0] = sign * math.hypot(vel_msg.linear.x - vel_msg.angular.z * self.steering_track / 2,
                                            vel_msg.angular.z * self.wheel_base / 2) - vel_steering_offset
            self.vel[1] = sign * math.hypot(vel_msg.linear.x + vel_msg.angular.z * self.steering_track / 2,
                                            vel_msg.angular.z * self.wheel_base / 2) + vel_steering_offset
            self.vel[2] = sign * math.hypot(vel_msg.linear.x - vel_msg.angular.z * self.steering_track / 2,
                                            vel_msg.angular.z * self.wheel_base / 2) - vel_steering_offset
            self.vel[3] = sign * math.hypot(vel_msg.linear.x + vel_msg.angular.z * self.steering_track / 2,
                                            vel_msg.angular.z * self.wheel_base / 2) + vel_steering_offset

            self.pos[0] = math.atan(vel_msg.angular.z * self.wheel_base /
                                    ((2 * vel_msg.linear.x) + vel_msg.angular.z * self.steering_track))
            self.pos[1] = math.atan(vel_msg.angular.z * self.wheel_base /
                                    ((2 * vel_msg.linear.x) - vel_msg.angular.z * self.steering_track))
            self.pos[2] = -self.pos[0]
            self.pos[3] = -self.pos[1]
        else:
            # Stop the robot if no command is given
            self.pos[:] = 0
            self.vel[:] = 0

        # Publish the position and velocity arrays
        pos_array = Float64MultiArray(data=self.pos)
        vel_array = Float64MultiArray(data=self.vel)
        self.pub_pos.publish(pos_array)
        self.pub_vel.publish(vel_array)


def main(args=None):
    rclpy.init(args=args)

    commander = Commander()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(commander)

    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    rate = commander.create_rate(20)
    try:
        while rclpy.ok():
            rate.sleep()
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
    executor_thread.join()


if __name__ == '__main__':
    main()

