#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import math
from geometry_msgs.msg import WrenchStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import time


class ForceSensor:
    def __init__(self):
        self.subscriber = rospy.Subscriber(
            "/force_sensor_data", WrenchStamped, self.callback
        )
        self.force = None

    def callback(self, data):
        self.force = data.wrench.force
        self.torque = data.wrench.torque

    def get_force(self):
        return self.force

    def get_torque(self):
        return self.torque


class HybridControl:
    def __init__(self, pub):
        rospy.init_node("hybrid_control", anonymous=True)
        self.joint_names = [
            "crane_x7_shoulder_fixed_part_pan_joint",
            "crane_x7_shoulder_revolute_part_tilt_joint",
            "crane_x7_upper_arm_revolute_part_twist_joint",
            "crane_x7_upper_arm_revolute_part_rotate_joint",
            "crane_x7_lower_arm_fixed_part_joint",
            "crane_x7_lower_arm_revolute_part_joint",
            "crane_x7_wrist_joint",
        ]
        self.pub = pub
        self.force_sensor = ForceSensor()
        self.prev_error = 0.0
        self.integral = 0.0
        self.integral_list = [0] * 10

        self.min_joint_value = 10
        self.max_joint_value = 80

        self.mode = 1
        self.offset = 0.2

        # 初期値
        if self.mode == 0:
            self.base_joint_value = 60
            target_joint_values = [
                math.radians(90),
                math.radians(self.base_joint_value),
                math.radians(0),
                math.radians(-2 * self.base_joint_value),
                math.radians(0),
                math.radians(self.base_joint_value),
                0.0,
            ]
        elif self.mode == 1:
            self.base_joint_value = 10
            target_joint_values = [
                math.radians(0),
                math.radians(-self.base_joint_value),
                math.radians(0),
                math.radians(-165 + 2 * self.base_joint_value),
                math.radians(0),
                math.radians(90 - self.base_joint_value),
                0.0,
            ]
        else:
            self.base_joint_value = 70
            alpha_r = math.asin(
                math.sin(math.radians(self.base_joint_value)) - self.offset
            )
            alpha = math.degrees(alpha_r)
            target_joint_values = [
                math.radians(90),
                math.radians(self.base_joint_value),
                math.radians(0),
                math.radians(-self.base_joint_value - alpha),
                math.radians(0),
                math.radians(alpha),
                0.0,
            ]
        self.send_joint_command(target_joint_values)

        

    def send_joint_command(self, joint_values):
        traj = JointTrajectory()
        traj.joint_names = self.joint_names
        point = JointTrajectoryPoint()
        point.positions = joint_values
        point.time_from_start = rospy.Duration(0.01)
        traj.points.append(point)
        # 複数回送信（コントローラが確実に受け取るため）
        for _ in range(2):
            self.pub.publish(traj)
            rospy.sleep(0.005)

    def control_loop(self):
        rate = rospy.Rate(100)
        EndF = False

        while not EndF and not rospy.is_shutdown():
            force = self.force_sensor.get_force()
            if force is not None:
                desired_force = 3.0
                current_force = -force.z
                force_error = desired_force - current_force
                control_signal = self.calculate_pid_control_signal(force_error)

                if control_signal > 10:
                    control_signal = 10
                if control_signal < -10:
                    control_signal = -10

                # 現在値は保持しておく
                if self.mode == 0:
                    self.base_joint_value = self.base_joint_value - control_signal
                    self.base_joint_value = max(self.min_joint_value, min(self.base_joint_value, self.max_joint_value))
                    target_joint_values = [
                        math.radians(0),
                        math.radians(self.base_joint_value),
                        math.radians(0),
                        math.radians(-2 * self.base_joint_value),
                        math.radians(0),
                        math.radians(self.base_joint_value),
                        0.0,
                    ]
                elif self.mode == 1:
                    self.base_joint_value = self.base_joint_value + control_signal
                    self.base_joint_value = max(self.min_joint_value, min(self.base_joint_value, self.max_joint_value))
                    target_joint_values = [
                        math.radians(0),
                        math.radians(-self.base_joint_value),
                        math.radians(0),
                        math.radians(-165 + 2 * self.base_joint_value),
                        math.radians(0),
                        math.radians(90 - self.base_joint_value),
                        0.0,
                    ]
                else:
                    self.base_joint_value = self.base_joint_value - control_signal
                    alpha_r = math.asin(
                        math.sin(math.radians(self.base_joint_value)) - self.offset
                    )
                    self.base_joint_value = max(self.min_joint_value, min(self.base_joint_value, self.max_joint_value))
                    alpha = math.degrees(alpha_r)
                    target_joint_values = [
                        math.radians(90),
                        math.radians(self.base_joint_value),
                        math.radians(0),
                        math.radians(-self.base_joint_value - alpha),
                        math.radians(0),
                        math.radians(alpha),
                        0.0,
                    ]

                self.send_joint_command(target_joint_values)

            rate.sleep()

        # 終了時にhomeに戻す例
        home_joint_values = [0.0] * 7
        self.send_joint_command(home_joint_values)

    def home(self):
        home_joint_values = [0.5, 0.5, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.send_joint_command(home_joint_values)

    def start_position(self):
        #初期値
        if self.mode == 0:
            self.base_joint_value = 60
            target_joint_values = [
                math.radians(90),
                math.radians(self.base_joint_value),
                math.radians(0),
                math.radians(-2 * self.base_joint_value),
                math.radians(0),
                math.radians(self.base_joint_value),
                0.0,
            ]
        elif self.mode == 1:
            self.base_joint_value = 10
            target_joint_values = [
                math.radians(0),
                math.radians(-self.base_joint_value),
                math.radians(0),
                math.radians(-165 + 2 * self.base_joint_value),
                math.radians(0),
                math.radians(90 - self.base_joint_value),
                0.0,
            ]
        else:
            self.base_joint_value = 70
            alpha_r = math.asin(
                math.sin(math.radians(self.base_joint_value)) - self.offset
            )
            alpha = math.degrees(alpha_r)
            target_joint_values = [
                math.radians(90),
                math.radians(self.base_joint_value),
                math.radians(0),
                math.radians(-self.base_joint_value - alpha),
                math.radians(0),
                math.radians(alpha),
                0.0,
            ]
        for i in range(9):
            self.send_joint_command(target_joint_values)
        rospy.sleep(1)

    def calculate_pid_control_signal(self, error):
        kp = 0.02
        ki = 0.00005
        kd = 0.03
        self.integral_list.pop(0)
        self.integral_list.append(error)
        self.integral = sum(self.integral_list)
        derivative = error - self.prev_error
        self.prev_error = error
        control_signal = kp * error + ki * self.integral + kd * derivative
        # end = time.time()
        # print("PID control signal {:.6f} at {:.6f}".format(control_signal, end))
        return control_signal


if __name__ == "__main__":
    pub = rospy.Publisher(
        "/crane_x7/arm_controller/command", JointTrajectory, queue_size=10
    )
    rospy.sleep(1.0)  # publisher準備待ち
    hc = HybridControl(pub)
    hc.control_loop()
