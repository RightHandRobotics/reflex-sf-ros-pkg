#!/usr/bin/env python
#
# RightHand Robotics code for interfacing with components of the ReFlex SF hand
#

from dynamixel_msgs.msg import JointState
from dynamixel_controllers.srv import TorqueEnable
from dynamixel_controllers.srv import SetSpeed
import rospy
from std_msgs.msg import Float64


class Motor(object):
    def __init__(self, name):
        '''
        Assumes that "name" is the name of the controller with a preceding
        slash, e.g. /reflex_sf_f1
        '''
        self.name = name[1:]
        self.zero_point = rospy.get_param(self.name + '/zero_point')
        self.ANGLE_RANGE = rospy.get_param(self.name + '/angle_range')
        self.JOINT_SPEED = rospy.get_param(self.name + '/joint_speed')
        self.MAX_SPEED = rospy.get_param(self.name + '/max_speed')
        self.FLIPPED = rospy.get_param(self.name + '/flipped')
        self.current_raw_angle = 0.0
        self.current_angle = 0.0
        self.load = 0
        self.OVERLOAD_THRESHOLD = 0.2  # overload threshold to avoid thermal issues
        self.TAU = 0.1  # time constant of lowpass filter
        self.angle_cmd_pub = rospy.Publisher(name + '/command', Float64, queue_size=10)
        self.set_speed_service = rospy.ServiceProxy(name + '/set_speed', SetSpeed)
        self.set_speed_service(self.JOINT_SPEED)
        self.torque_enable_service = rospy.ServiceProxy(name + '/torque_enable', TorqueEnable)
        self.torque_enabled = True
        self.sub = rospy.Subscriber(name + '/state', JointState, self.receive_state_cb)

    def set_local_motor_zero_point(self):
        self.zero_point = self.current_raw_angle
        rospy.set_param(self.name + '/zero_point', self.current_raw_angle)

    def get_current_raw_angle(self):
        return self.current_raw_angle

    def get_current_angle(self):
        return self.current_angle

    def set_raw_motor_angle(self, goal_pos):
        self.angle_cmd_pub.publish(goal_pos)

    def set_motor_angle(self, goal_pos):
        '''
        Bounds the given position command and sets it to the motor
        '''
        self.angle_cmd_pub.publish(self.check_motor_angle_command(goal_pos))

    def check_motor_angle_command(self, angle_command):
        '''
        Returns given command if within the allowable range, returns bounded command if out of range
        '''
        angle_command = self.correct_motor_offset(angle_command)
        if self.FLIPPED:
            bounded_command = max(min(angle_command, self.zero_point), self.zero_point - self.ANGLE_RANGE)
        else:
            bounded_command = min(max(angle_command, self.zero_point), self.zero_point + self.ANGLE_RANGE)
        return bounded_command

    def set_motor_velocity(self, goal_vel):
        '''
        Sets speed and commands finger in or out based on sign of velocity
        '''
        self.set_motor_speed(goal_vel)
        if goal_vel > 0.0:
            self.angle_cmd_pub.publish(self.check_motor_angle_command(self.ANGLE_RANGE))
        elif goal_vel < 0.0:
            self.angle_cmd_pub.publish(self.check_motor_angle_command(0.0))

    def set_motor_speed(self, goal_speed):
        '''
        Bounds the given position command and sets it to the motor
        '''
        self.set_speed_service(self.check_motor_speed_command(goal_speed))

    def check_motor_speed_command(self, vel_command):
        '''
        Returns absolute of given command if within the allowable range, returns bounded command if out of range
        Always returns positive (speed)
        '''
        bounded_command = min(abs(vel_command), self.MAX_SPEED)
        return bounded_command

    def reset_motor_speed(self):
        '''
        Resets speed to default
        '''
        self.set_speed_service(self.JOINT_SPEED)

    def correct_motor_offset(self, angle_command):
        '''
        Adjusts for the zero point offset
        '''
        if self.FLIPPED:
            return self.zero_point - angle_command
        else:
            return self.zero_point + angle_command

    def enable_torque(self):
        self.torque_enabled = True
        self.torque_enable_service(True)

    def disable_torque(self):
        self.torque_enabled = False
        self.torque_enable_service(False)

    def loosen_if_overloaded(self, load):
        if abs(load) > self.OVERLOAD_THRESHOLD:
            rospy.logwarn("Motor %s overloaded at %f, loosening" % (self.name, load))
            self.loosen()

    def tighten(self, tighten_angle=0.05):
        '''
        Takes the given angle offset in radians and tightens the motor
        '''
        if self.FLIPPED:
            tighten_angle *= -1
        self.set_raw_motor_angle(self.current_raw_angle + tighten_angle)

    def loosen(self, loosen_angle=0.05):
        '''
        Takes the given angle offset in radians and loosens the motor
        '''
        if self.FLIPPED:
            loosen_angle *= -1
        self.set_raw_motor_angle(self.current_raw_angle - loosen_angle)

    def receive_state_cb(self, data):
        self.current_raw_angle = data.current_pos
        self.load = self.TAU * data.load + (1 - self.TAU) * self.load  # Rolling filter of noisy data
        if self.FLIPPED:
            self.current_angle = self.zero_point - self.current_raw_angle
        else:
            self.current_angle = self.current_raw_angle - self.zero_point
        self.loosen_if_overloaded(self.load)
