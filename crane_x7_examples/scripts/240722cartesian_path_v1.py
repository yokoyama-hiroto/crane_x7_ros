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
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
import rosnode
from tf.transformations import quaternion_from_euler
from moveit_msgs.msg import JointConstraint, Constraints

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
            target_pose.orientation = target_orientation
            way_points.append(target_pose)

    path, fraction = arm.compute_cartesian_path(way_points, eef_step, jump_threshold, avoid_collisions)
    arm.execute(path)

def main():
    # gripper.set_joint_value_target([0.9, 0.9])
    # gripper.go()

    arm.set_named_target("home")
    arm.go()

    # to lock the joint 7
    current_joint_value = arm.get_current_joint_values()
    # fixed_joint_value = current_joint_value[6]
    # joint_constraint = JointConstraint()
    # joint_constraint.joint_name = arm.get_joints()[6]
    # joint_constraint.position = fixed_joint_value
    # joint_constraint.tolerance_above = 0.0
    # joint_constraint.tolerance_below = 0.0
    # joint_constraint.weight = 1.0

    # constraints = Constraints()
    # constraints.joint_constraints.append(joint_constraint)

    # arm.set_path_constraints(constraints)



    # 座標(x=0.3, y=0.0, z=0.1)を中心に、XY平面上に半径0.1 mの円を3回描くように手先を動かす
    follow_rectangle_path(center_position=Point(0.3, 0.0, 0.1), radius=0.1, repeat=4)

    arm.set_named_target("home")
    arm.go()

if __name__ == '__main__':
    rospy.init_node("cartesian_path_example")
    robot = moveit_commander.RobotCommander()
    arm = moveit_commander.MoveGroupCommander("arm")
    arm.set_max_velocity_scaling_factor(0.1)
    arm.set_max_acceleration_scaling_factor(1.0)
    # gripper = moveit_commander.MoveGroupCommander("gripper")

    try:
        if not rospy.is_shutdown():
            main()
    except rospy.ROSInterruptException:
        pass
