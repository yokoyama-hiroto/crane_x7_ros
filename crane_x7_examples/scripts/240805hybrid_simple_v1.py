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
        self.integral_list = [0]*10

        self.mode = 2
        self.offset = 0.2

        

        # 初期位置に移動
        # initial_position = Pose()
        # initial_position.position.x = self.path[0][0]
        # initial_position.position.y = self.path[0][1]
        # initial_position.position.z = self.path[0][2]
        # q = quaternion_from_euler( 0.0, math.radians(0), 0.0 )
        # initial_position.orientation = Quaternion(q[0], q[1], q[2], q[3])

        self.arm.clear_path_constraints()


        target_joint_values = self.arm.get_current_joint_values()

        if self.mode==0:
            self.base_joint_value = 60
            target_joint_values[0] = math.radians(90) 
            target_joint_values[1] = math.radians(self.base_joint_value ) 
            target_joint_values[2] = math.radians(0) 
            target_joint_values[3] = math.radians(-2 * self.base_joint_value) 
            target_joint_values[4] = math.radians(0) 
            target_joint_values[5] = math.radians(self.base_joint_value) 
        elif self.mode==1:
            self.base_joint_value = 15
            target_joint_values[0] = math.radians(0) 
            target_joint_values[1] = math.radians(-self.base_joint_value ) 
            target_joint_values[2] = math.radians(0) 
            target_joint_values[3] = math.radians(-180 + 2 * self.base_joint_value) 
            target_joint_values[4] = math.radians(0) 
            target_joint_values[5] = math.radians(90 - self.base_joint_value) 
        else:
            self.base_joint_value = 70
            alpha_r = math.asin(math.sin(math.radians(self.base_joint_value)) - self.offset)
            alpha = math.degrees(alpha_r)
            target_joint_values[0] = math.radians(90) 
            target_joint_values[1] = math.radians(self.base_joint_value ) 
            target_joint_values[2] = math.radians(0) 
            target_joint_values[3] = math.radians(-self.base_joint_value -alpha) 
            target_joint_values[4] = math.radians(0) 
            target_joint_values[5] = math.radians(alpha)            

        self.arm.set_joint_value_target(target_joint_values)
        self.arm.go()

        # self.arm.set_pose_target(initial_position)
        # self.arm.go(wait=True)
        # eef_step=0.01
        # jump_threshold=0.0
        # avoid_collisions=True
        # path, fraction = self.arm.compute_cartesian_path(initial_position, eef_step, jump_threshold, avoid_collisions)
        # self.arm.execute(path)
        # self.arm.set_pose_target(initial_position)
        # self.arm.go()


        

    def control_loop(self):
        rate = rospy.Rate(20)

        # 関節7を固定
        # current_joint_value = self.arm.get_current_joint_values()
        # fixed_joint_value = current_joint_value[6]
        # joint_constraint = JointConstraint()
        # joint_constraint.joint_name = self.arm.get_joints()[7]
        # joint_constraint.position = fixed_joint_value
        # joint_constraint.tolerance_above = 0.0
        # joint_constraint.tolerance_below = 0.0
        # joint_constraint.weight = 1.0

        # rospy.loginfo(self.arm.get_joints()[7])
        # rospy.loginfo(current_joint_value[6])
        # エンドエフェクタのロール方向の許容を大きくし、他の方向は小さくする
        # orientation_constraint = OrientationConstraint()
        # # orientation_constraint.link_name = arm.get_end_effector_link()
        # orientation_constraint.link_name = "crane_x7_wrist_joint"
        # orientation_constraint.header.frame_id = "base_link"
        # #orientation_constraint.header.frame_id = self.arm.get_planning_frame()
        # orientation_constraint.orientation.w = 1.0
        # orientation_constraint.absolute_x_axis_tolerance = 0.1
        # orientation_constraint.absolute_y_axis_tolerance = 0.1
        # orientation_constraint.absolute_z_axis_tolerance = 3.14
        # orientation_constraint.weight = 1.0
        
        # constraints = Constraints()
        # # constraints.joint_constraints.append(joint_constraint)
        # constraints.orientation_constraints.append(orientation_constraint)
        # self.arm.set_path_constraints(constraints)

        # way_point = 10
        # eef_step=0.01
        # jump_threshold=0.0
        EndF = False



        while not EndF and not rospy.is_shutdown():
            force = self.force_sensor.get_force()
            if force is not None:
                force = self.force_sensor.get_force()
                desired_force = 3.0
                current_force = -force.z
                force_error = desired_force - current_force
                # force_error = 0.01
                # rospy.loginfo(force_error)
                control_signal = self.calculate_pid_control_signal(force_error)

                #########
                target_joint_values = self.arm.get_current_joint_values()

                if control_signal > 10:
                    control_signal = 10
                
                if control_signal < -10:
                    control_signal = -10
                    


                if self.mode==0:
                    self.base_joint_value = self.base_joint_value - control_signal
                    target_joint_values[0] = math.radians(0) 
                    target_joint_values[1] = math.radians(self.base_joint_value ) 
                    target_joint_values[2] = math.radians(0) 
                    target_joint_values[3] = math.radians(-2 * self.base_joint_value) 
                    target_joint_values[4] = math.radians(0) 
                    target_joint_values[5] = math.radians(self.base_joint_value) 
                elif self.mode==1:
                    self.base_joint_value = self.base_joint_value + control_signal
                    target_joint_values[0] = math.radians(0) 
                    target_joint_values[1] = math.radians(-self.base_joint_value ) 
                    target_joint_values[2] = math.radians(0) 
                    target_joint_values[3] = math.radians(-180 +2 * self.base_joint_value) 
                    target_joint_values[4] = math.radians(0) 
                    target_joint_values[5] = math.radians(90 - self.base_joint_value)
                else:
                    self.base_joint_value = self.base_joint_value - control_signal
                    alpha_r = math.asin(math.sin(math.radians(self.base_joint_value)) - self.offset)
                    alpha = math.degrees(alpha_r)
                    target_joint_values[0] = math.radians(90) 
                    target_joint_values[1] = math.radians(self.base_joint_value ) 
                    target_joint_values[2] = math.radians(0) 
                    target_joint_values[3] = math.radians(-self.base_joint_value-alpha) 
                    target_joint_values[4] = math.radians(0) 
                    target_joint_values[5] = math.radians(alpha)   



                self.arm.set_joint_value_target(target_joint_values)
                self.arm.go()
                #########
            
                # rospy.loginfo(current_position)
                # rospy.loginfo(desired_position)

            rate.sleep()

        

        self.arm.clear_path_constraints()
        self.arm.set_named_target("home")
        self.arm.go()

    def calculate_pid_control_signal(self, error):
        if self.mode==0:
            kp = 0.6
            ki = 0.0005
        elif self.mode==1:
            kp = 0.6
            ki = 0.0005
        else:
            kp = 0.6
            ki = 0.0005           

        # kd = 0.001
        self.integral_list.pop(0)
        self.integral_list.append(error)
        self.integral = sum(self.integral_list)
        #derivative = error - self.prev_error
        self.prev_error = error

        control_signal = kp * error + ki * self.integral #+ kd * derivative
        return control_signal

if __name__ == '__main__':
    hc = HybridControl()
    hc.control_loop()
