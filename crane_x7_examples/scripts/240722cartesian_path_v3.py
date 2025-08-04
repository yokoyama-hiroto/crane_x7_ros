#! /usr/bin/env python
# -*- coding: utf-8 -*-

# Copyright 2020 RT Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rospy
import moveit_commander
import math
from geometry_msgs.msg import WrenchStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
import rosnode
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

def follow_rectangle_path(
    center_position=Point(0.3, 0.0, 0.1), radius=0.1,
    num_of_waypoints=100, repeat=1,
    eef_step=0.01, jump_threshold=0.0, avoid_collisions=True):
    # 円軌道を生成するためのコード
    way_points = []
    # オイラー角でアームを鉛直下に向けている（デフォルトが垂直上を向いているため）
    q = quaternion_from_euler( 0.0, math.radians(0.0), 0.0 )
    target_orientation = Quaternion(q[0], q[1], q[2], q[3])
    
    l = 0.2
    w = 0.2
    d = 0.45
    path = [
            [0, 0, d],
            [0, l, d],
            [w, l, d],
            [w, 0, d]
        ]
    for j in range(repeat):
        for i in range(num_of_waypoints):
            delta = i / float(num_of_waypoints)
            target_pose = Pose()
    
            
            if j==0:
                target_pose.position.x = l * delta
                target_pose.position.y = 0
            elif j==1:
                target_pose.position.x = l 
                target_pose.position.y = w * delta
            elif j==2:
                target_pose.position.x = l * (1 - delta)
                target_pose.position.y = w 
            else:
                target_pose.position.x = 0
                target_pose.position.y = w * (1 - delta)
                
            target_pose.position.z = d
            #target_pose.orientation = target_orientation
            way_points.append(target_pose)

    path, fraction = arm.compute_cartesian_path(way_points, eef_step, jump_threshold, avoid_collisions)
    arm.execute(path)

def main():
    # gripper.set_joint_value_target([0.9, 0.9])
    # gripper.go()

    arm.set_named_target("home")
    arm.go()

    # 関節7を固定
    current_joint_value = arm.get_current_joint_values()
    fixed_joint_value = current_joint_value[6]
    joint_constraint = JointConstraint()
    joint_constraint.joint_name = arm.get_joints()[6]
    joint_constraint.position = fixed_joint_value
    joint_constraint.tolerance_above = 0.0
    joint_constraint.tolerance_below = 0.0
    joint_constraint.weight = 1.0

    rospy.loginfo(arm.get_joints()[7])
    rospy.loginfo(current_joint_value[6])
    
    # エンドエフェクタのロール方向の許容を大きくし、他の方向は小さくする
    #orientation_constraint = OrientationConstraint()
    # orientation_constraint.link_name = arm.get_end_effector_link()
    #orientation_constraint.link_name = "crane_x7_wrist_joint"
    # orientation_constraint.header.frame_id = "base_link"
    #orientation_constraint.header.frame_id = arm.get_planning_frame()
    #orientation_constraint.orientation.w = 1.0
    #orientation_constraint.absolute_x_axis_tolerance = 0.1
    #orientation_constraint.absolute_y_axis_tolerance = 0.1
    #orientation_constraint.absolute_z_axis_tolerance = 0.1
    #orientation_constraint.weight = 1.0
    
    constraints = Constraints()
    constraints.joint_constraints.append(joint_constraint)
    #constraints.orientation_constraints.append(orientation_constraint)

    arm.set_path_constraints(constraints)

    

    # 座標(x=0.3, y=0.0, z=0.1)を中心に、XY平面上に半径0.1 mの円を3回描くように手先を動かす
    follow_rectangle_path(center_position=Point(0.3, 0.0, 0.1), radius=0.1, repeat=4)
    
    while not rospy.is_shutdown():
        rate = rospy.Rate(10)
        force = force_sensor.get_force()
        if force is not None:
            desired_force = -5.0
            current_force = force.z
            # force_error = desired_force - current_force
            force_error = 0
            control_signal = calculate_pid_control_signal(force_error)

            desired_position = Pose()
            current_position = arm.get_current_pose().pose.position
            desired_position.position = current_position
            desired_position.position.z += control_signal

            # 四角形の軌跡に従って目標位置を更新
            target_position = path[path_index]
            desired_position.position.x = target_position[0]
            desired_position.position.y = target_position[1]

            arm.set_pose_target(desired_position)
            
            rospy.loginfo(arm.get_current_pose())
            rospy.loginfo(current_position)
            rospy.loginfo(desired_position)
            if arm.go(wait=True):
                # 次の目標位置に移動
                path_index = (path_index + 1) % len(path)

        rate.sleep()

	
    arm.clear_path_constraints()

    arm.set_named_target("home")
    arm.go()


    def calculate_pid_control_signal(self, error):
        kp = 0.5
        ki = 0.05
        kd = 0.001

        self.integral += error
        derivative = error - self.prev_error
        self.prev_error = error

        control_signal = kp * error + ki * self.integral + kd * derivative
        return control_signal


if __name__ == '__main__':
    rospy.init_node("cartesian_path_example")
    robot = moveit_commander.RobotCommander()
    arm = moveit_commander.MoveGroupCommander("arm")
    arm.set_max_velocity_scaling_factor(0.1)
    arm.set_max_acceleration_scaling_factor(1.0)

    force_sensor = ForceSensor()
    prev_error = 0.0
    integral = 0.0

    try:
        if not rospy.is_shutdown():
            main()
    except rospy.ROSInterruptException:
        pass
