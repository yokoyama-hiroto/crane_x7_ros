#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import moveit_commander
import math
from geometry_msgs.msg import WrenchStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
import sys
from tf.transformations import quaternion_from_euler
from moveit_msgs.msg import JointConstraint, Constraints, OrientationConstraint

class ForceSensor:
    def __init__(self):
        # rospy.init_node('force_sensor_listener')
        self.subscriber = rospy.Subscriber('/force_sensor_data', WrenchStamped, self.callback)
        self.force = None

    def callback(self, data):
        self.force = data.wrench.force
        self.torque = data.wrench.torque

    def get_force(self):
        return self.force

    def get_torque(self):
        return self.torque

class HybridControl:
    def __init__(self):
        # moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('hybrid_control', anonymous=True)
        self.robot = moveit_commander.RobotCommander()
        self.arm = moveit_commander.MoveGroupCommander("arm")
        # self.gripper = moveit_commander.MoveGroupCommander("gripper")

        self.arm.set_max_velocity_scaling_factor(0.1)
        self.arm.set_max_acceleration_scaling_factor(1.0)
        
        self.force_sensor = ForceSensor()
        self.prev_error = 0.0
        self.integral = 0.0

        # 四角形の軌跡の頂点を定義(天井用)
        self.path = [
            [0.0, 0.0, 0.4],
            [0.0, 0.2, 0.4],
            [0.2, 0.2, 0.4],
            [0.2, 0.0, 0.4],
            [0.0, 0.0, 0.4]
        ]
        # 四角形の軌跡の頂点を定義(壁面用)
        # self.path = [
        #     [0.3, 0, 0],
        #     [0.3, 0.2, 0],
        #     [0.3, 0.2, 0.2],
        #     [0.3, 0, 0.2]
        # ]
        self.path_index = 0

        # 初期位置に移動
        initial_position = Pose()
        initial_position.position.x = self.path[0][0]
        initial_position.position.y = self.path[0][1]
        initial_position.position.z = self.path[0][2]
        q = quaternion_from_euler( 0.0, math.radians(0), 0.0 )
        initial_position.orientation = Quaternion(q[0], q[1], q[2], q[3])

        self.arm.clear_path_constraints()

        # self.arm.set_pose_target(initial_position)
        # self.arm.go(wait=True)
        eef_step=0.01
        jump_threshold=0.0
        avoid_collisions=True
        # path, fraction = self.arm.compute_cartesian_path(initial_position, eef_step, jump_threshold, avoid_collisions)
        # self.arm.execute(path)
        self.arm.set_pose_target(initial_position)
        self.arm.go()


        

    def control_loop(self):
        rate = rospy.Rate(10)

        # 関節7を固定
        current_joint_value = self.arm.get_current_joint_values()
        fixed_joint_value = current_joint_value[6]
        joint_constraint = JointConstraint()
        joint_constraint.joint_name = self.arm.get_joints()[7]
        joint_constraint.position = fixed_joint_value
        joint_constraint.tolerance_above = 0.0
        joint_constraint.tolerance_below = 0.0
        joint_constraint.weight = 1.0

        rospy.loginfo(self.arm.get_joints()[7])
        rospy.loginfo(current_joint_value[6])
        # エンドエフェクタのロール方向の許容を大きくし、他の方向は小さくする
        orientation_constraint = OrientationConstraint()
        # orientation_constraint.link_name = arm.get_end_effector_link()
        orientation_constraint.link_name = "crane_x7_wrist_joint"
        orientation_constraint.header.frame_id = "base_link"
        #orientation_constraint.header.frame_id = self.arm.get_planning_frame()
        orientation_constraint.orientation.w = 1.0
        orientation_constraint.absolute_x_axis_tolerance = 0.1
        orientation_constraint.absolute_y_axis_tolerance = 0.1
        orientation_constraint.absolute_z_axis_tolerance = 3.14
        orientation_constraint.weight = 1.0
        
        constraints = Constraints()
        # constraints.joint_constraints.append(joint_constraint)
        constraints.orientation_constraints.append(orientation_constraint)
        # self.arm.set_path_constraints(constraints)


        way_point = 20
        eef_step=0.01
        jump_threshold=0.0
        EndF = False

        while not EndF and not rospy.is_shutdown():
            force = self.force_sensor.get_force()
            if force is not None:
                for i in range(way_point):
                    force = self.force_sensor.get_force()
                    desired_force = 5.0
                    current_force = -force.z
                    force_error = desired_force - current_force
                    # # force_error = 0.01
                    # rospy.loginfo(force_error)
                    control_signal = self.calculate_pid_control_signal(force_error)

                    desired_position = Pose()
                    current_position = self.arm.get_current_pose().pose
                    desired_position.position = current_position.position
                    desired_position.position.z += control_signal

                    if desired_position.position.z > 0.5:
                        desired_position.position.z = 0.5

                    if desired_position.position.z < 0.38:
                        desired_position.position.z = 0.38

                    # rospy.loginfo(desired_position.position.z)

                    desired_position.orientation.w = 1.0
                    segment = (float(i)+1)/float(way_point)


                    # target_position = (1.0-segment)*float(self.path[self.path_index]) + segment*self.path[self.path_index+1.0]
                    # desired_position.position.x = target_position[0]
                    # desired_position.position.y = target_position[1]
                    desired_position.position.x =  (1.0-segment)*self.path[self.path_index][0] + segment*self.path[self.path_index+1][0]
                    desired_position.position.y = (1.0-segment)*self.path[self.path_index][1] + segment*self.path[self.path_index+1][1]
                    
                    self.arm.set_pose_target(desired_position)

                    waypoint_input = []
                    # waypoint_input.append(current_position)
                    waypoint_input.append(desired_position)

                    path, fraction = self.arm.compute_cartesian_path(waypoint_input, eef_step, jump_threshold, avoid_collisions=True)
                    self.arm.execute(path)
                
                    # rospy.loginfo(current_position)
                    # rospy.loginfo(desired_position)
                # if self.arm.execute(path,wait=True):
                #     # 次の目標位置に移動
                #     self.path_index = (self.path_index + 1) % len(self.path)
                    # rospy.loginfo(desired_position.orientation)
                self.path_index = (self.path_index + 1)
                if self.path_index == 4:
                    EndF = True
                    break

            rate.sleep()

        

        self.arm.clear_path_constraints()
        self.arm.set_named_target("home")
        self.arm.go()

    def calculate_pid_control_signal(self, error):
        kp = 0.1
        ki = 0.01
        # kd = 0.001

        self.integral += error
        #derivative = error - self.prev_error
        self.prev_error = error

        control_signal = kp * error + ki * self.integral #+ kd * derivative
        return control_signal

if __name__ == '__main__':
    hc = HybridControl()
    hc.control_loop()
