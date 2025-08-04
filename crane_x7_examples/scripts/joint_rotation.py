#! /usr/bin/env python
# -*- coding: utf-8 -*-

# Copyright 2019 RT Corporation
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
import math
import moveit_commander
# import keyboard


def main():
    arm = moveit_commander.MoveGroupCommander("arm")
    # 駆動速度を調整する
    speed = 0.125
    # (0:horizontally, 1：vetrically, 2: forward)
    pattern = 2
    tgt_count = 10
    arm.set_max_velocity_scaling_factor(speed)
    arm.set_max_acceleration_scaling_factor(1.0)

    # SRDFに定義されている"vertical"の姿勢にする
    # すべてのジョイントの目標角度が0度になる

    print("move to the start position in 30 s ")
    target_joint_values = arm.get_current_joint_values()
    target_joint_values[0] = math.radians(0) 
    target_joint_values[1] = math.radians(0) 
    target_joint_values[2] = math.radians(0) 
    target_joint_values[3] = math.radians(-140) 
    target_joint_values[4] = math.radians(0) 
    target_joint_values[5] = math.radians(0)
    arm.set_joint_value_target(target_joint_values)
    arm.go()

    # while True:
    #     if keyboard.is_pressed('h'):
    #         break
    #     rospy.sleep(0.5)
    rospy.sleep(10)
    arm.set_named_target("home")
    arm.go()

    # 目標角度と実際の角度を確認
    print("joint_value_target (radians):")
    print(arm.get_joint_value_target())
    print("current_joint_values (radians):")
    print(arm.get_current_joint_values())



#    joint_angle = math.radians(-45)
#     for i in range(7):
#         target_joint_values[i] = joint_angle
#         arm.set_joint_value_target(target_joint_values)
#         arm.go()
#         print(str(i) + "-> joint_value_target (degrees):"
#               + str(math.degrees(arm.get_joint_value_target()[i]))
#               + ", current_joint_values (degrees):"
#               + str(math.degrees(arm.get_current_joint_values()[i]))
#         )
    target_joint_values = arm.get_current_joint_values()
    if pattern==0:
        target_joint_values[1] = math.radians(-90)   
        target_joint_values[3] = math.radians(-90)   
    elif pattern==1:
        target_joint_values[0] = math.radians(0)
        target_joint_values[1] = math.radians(0) 
        target_joint_values[3] = math.radians(-90)
    else:
        target_joint_values[0] = math.radians(90) 
        target_joint_values[1] = math.radians(0) 
        target_joint_values[3] = math.radians(-90) 
        
    target_joint_values[2] = math.radians(0) 
    target_joint_values[4] = math.radians(0) 
    target_joint_values[5] = math.radians(0) 
    arm.set_joint_value_target(target_joint_values)
    arm.go()
    rospy.sleep(1)

    flag = True
    count = 0

    
    while not rospy.is_shutdown():
        # 現在角度をベースに、目標角度を作成する
        target_joint_values = arm.get_current_joint_values()


        
        if pattern==0:
            if flag:        
                joint_angle = math.radians(-45)
                flag = False
            else:
                joint_angle = math.radians(45)
                flag = True
            target_joint_values[0] = joint_angle
            target_joint_values[1] = math.radians(-90)   
            target_joint_values[3] = math.radians(-90)     
        else:
            if flag:        
                joint_angle = math.radians(-90)
                flag = False
            else:
                joint_angle = math.radians(0)
                flag = True
            target_joint_values[1] = joint_angle      
            target_joint_values[3] = math.radians(-90) 

        arm.set_joint_value_target(target_joint_values)
        arm.go()
        count = count + 1
        print(str(0) + "-> joint_value_target (degrees):"
                + str(math.degrees(arm.get_joint_value_target()[pattern]))
                + ", current_joint_values (degrees):"
                + str(math.degrees(arm.get_current_joint_values()[pattern]))
        )

        if count >= tgt_count:
            break
        
        # rospy.sleep(0.25)
    
    # 垂直に戻す
    arm.set_named_target("home")
    arm.go()


if __name__ == '__main__':
    rospy.init_node("joint_values_example")

    try:
        if not rospy.is_shutdown():
            main()
    except rospy.ROSInterruptException:
        pass
