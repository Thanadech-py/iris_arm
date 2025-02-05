#!/usr/bin/env python3

import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import Joy

class ArmJoystickController:
    def __init__(self):
        rospy.init_node("joystick_arm_control")

        # Publisher for arm joint commands
        self.pub = rospy.Publisher('/arm_controller/command', JointTrajectory, queue_size=10)

        # Subscriber to joystick topic
        rospy.Subscriber('/joy', Joy, self.joy_callback)

        # Joint names
        self.joint_names = ['hip', 'shoulder', 'elbow', 'wrist', 'l_g_base', 'r_g_base']
        self.joint_dict = {'hip': 0, 'shoulder': 1, 'elbow': 2, 'wrist': 3}
        
        # Initialize joint positions
        self.jt = JointTrajectory()
        self.jt.header.frame_id = "base_link"
        self.jt.joint_names = self.joint_names
        self.jt.points.append(JointTrajectoryPoint())
        self.jt.points[0].positions = [0.0] * len(self.joint_names)

        self.gripper_open = True  # Gripper state
        self.joystick_axes = [0] * 6  # Store joystick axis values
        self.joystick_buttons = [0] * 12  # Store button values

        rospy.loginfo("Joystick control for robotic arm initialized.")

    def joy_callback(self, data):
        """ Callback function for joystick input """
        self.joystick_axes = data.axes
        self.joystick_buttons = data.buttons
        self.update_arm_position()

    def update_arm_position(self):
        """ Updates joint positions based on joystick input """

        # Mapping joystick axes to joint movements
        self.jt.points[0].positions[0] += self.joystick_axes[0] * 0.02  # Hip (Left Stick X)
        self.jt.points[0].positions[1] += self.joystick_axes[1] * 0.02  # Shoulder (Left Stick Y)
        self.jt.points[0].positions[2] += self.joystick_axes[3] * 0.02  # Elbow (Right Stick Y)
        self.jt.points[0].positions[3] += self.joystick_axes[2] * 0.02  # Wrist (Right Stick X)

        # Gripper control with button press
        if self.joystick_buttons[0]:  # Button A / X
            if not self.gripper_open:
                rospy.loginfo("Opening gripper...")
                self.jt.points[0].positions[4] = 0.0
                self.jt.points[0].positions[5] = 0.0
                self.gripper_open = True
        elif self.joystick_buttons[1]:  # Button B / Circle
            if self.gripper_open:
                rospy.loginfo("Closing gripper...")
                self.jt.points[0].positions[4] = 0.5236
                self.jt.points[0].positions[5] = -0.5236
                self.gripper_open = False

        # Ensure trajectory timing
        self.jt.points[0].time_from_start = rospy.Duration(0.5)

        # Publish the trajectory command
        self.pub.publish(self.jt)

    def run(self):
        """ Runs the ROS node """
        rospy.spin()

if __name__ == "__main__":
    controller = ArmJoystickController()
    controller.run()
