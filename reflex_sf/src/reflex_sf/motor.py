from dynamixel_msgs.msg import JointState
from dynamixel_controllers.srv import TorqueEnable
from dynamixel_controllers.srv import SetSpeed
import rospy
from std_msgs.msg import Float64

import reflex_msgs.msg


class Motor(object):
    def __init__(self, name):
        '''
        Assumes that "name" is the name of the controller with a preceding
        slash, e.g. /reflex_sf_f1
        '''
        self.namespace = name[1:]
        self.zero_point = rospy.get_param(self.namespace + '/zero_point')
        self.DEFAULT_MOTOR_SPEED = rospy.get_param(self.namespace + '/default_motor_speed')
        self.MAX_MOTOR_SPEED = rospy.get_param(self.namespace + '/max_motor_speed')
        self.MAX_MOTOR_TRAVEL = rospy.get_param(self.namespace + '/max_motor_travel')
        self.MOTOR_TO_JOINT_INVERTED = rospy.get_param(self.namespace + '/motor_to_joint_inverted')
        self.MOTOR_TO_JOINT_GEAR_RATIO = rospy.get_param(self.namespace + '/motor_to_joint_gear_ratio')
        self.OVERLOAD_THRESHOLD = rospy.get_param(self.namespace + '/overload_threshold')
        self.motor_msg = reflex_msgs.msg.Motor()
        self.motor_cmd_pub = rospy.Publisher(name + '/command', Float64, queue_size=10)
        self.in_control_torque_mode = False
        self.torque_cmd = 0.0
        self.previous_load_control_output = 0.0
        self.previous_load_control_error = 0.0
        self.set_speed_service = rospy.ServiceProxy(name + '/set_speed', SetSpeed)
        self.reset_motor_speed()
        self.torque_enable_service = rospy.ServiceProxy(name + '/torque_enable', TorqueEnable)
        self.torque_enable_service(True)
        self.state_subscriber = rospy.Subscriber(name + '/state', JointState, self.receive_state_cb)

    def set_local_motor_zero_point(self):
        self.zero_point = self.motor_msg.raw_angle
        rospy.set_param(self.namespace + '/zero_point', self.motor_msg.raw_angle)

    def get_current_raw_motor_angle(self):
        return self.motor_msg.raw_angle

    def get_current_joint_angle(self):
        return self.motor_msg.joint_angle

    def get_load(self):
        return self.motor_msg.load

    def get_motor_msg(self):
        return self.motor_msg

    def set_raw_motor_angle(self, goal_pos):
        self.motor_cmd_pub.publish(goal_pos)

    def set_motor_angle(self, goal_pos):
        '''
        Bounds the given position command and sets it to the motor
        '''
        goal_pos *= self.MOTOR_TO_JOINT_GEAR_RATIO  # Goes from joint radians to motor radians
        self.motor_cmd_pub.publish(self.check_motor_angle_command(goal_pos))

    def check_motor_angle_command(self, angle_command):
        '''
        Returns given command if within the allowable range, returns bounded command if out of range
        '''
        angle_command = self.correct_motor_offset(angle_command)
        if self.MOTOR_TO_JOINT_INVERTED:
            bounded_command = max(min(angle_command, self.zero_point),
                                  self.zero_point - self.MAX_MOTOR_TRAVEL * self.MOTOR_TO_JOINT_GEAR_RATIO)
        else:
            bounded_command = min(max(angle_command, self.zero_point),
                                  self.zero_point + self.MAX_MOTOR_TRAVEL * self.MOTOR_TO_JOINT_GEAR_RATIO)
        return bounded_command

    def set_motor_speed(self, goal_speed):
        '''
        Bounds the given position command and sets it to the motor
        '''
        self.set_speed_service(self.check_motor_speed_command(goal_speed))

    def check_motor_speed_command(self, goal_speed):
        '''
        Returns absolute of given command if within the allowable range, returns bounded command if out of range
        Always returns positive (speed)
        '''
        bounded_command = min(abs(goal_speed), self.MAX_MOTOR_SPEED)
        return bounded_command

    def reset_motor_speed(self):
        '''
        Resets speed to default
        '''
        self.set_speed_service(self.DEFAULT_MOTOR_SPEED)

    def set_motor_velocity(self, goal_vel):
        '''
        Sets speed and commands finger in or out based on sign of velocity
        '''
        self.set_motor_speed(goal_vel)
        if goal_vel > 0.0:
            max_command = self.MAX_MOTOR_TRAVEL * self.MOTOR_TO_JOINT_GEAR_RATIO
            self.motor_cmd_pub.publish(self.check_motor_angle_command(max_command))
        elif goal_vel < 0.0:
            self.motor_cmd_pub.publish(self.check_motor_angle_command(0.0))

    def correct_motor_offset(self, angle_command):
        '''
        Adjusts for the zero point offset
        '''
        if self.MOTOR_TO_JOINT_INVERTED:
            return self.zero_point - angle_command
        else:
            return self.zero_point + angle_command

    def enable_torque_control(self):
        self.in_control_torque_mode = True
        self.previous_load_control_output = self.get_current_joint_angle()
        self.previous_load_control_error = 0.0

    def disable_torque_control(self):
        self.in_control_torque_mode = False

    def set_torque_cmd(self, torque_cmd):
        '''
        Bounds the given goal load and sets it as the goal
        '''
        self.torque_cmd = min(max(torque_cmd, 0.0), self.OVERLOAD_THRESHOLD)

    def enable_torque(self):
        self.torque_enable_service(True)

    def disable_torque(self):
        self.torque_enable_service(False)

    def control_torque(self, current_torque):
        current_error = self.torque_cmd - current_torque
        k = 3.0 * 0.025  # Compensator gain - higher gain has faster response and is more unstable
        output = self.previous_load_control_output + k * (current_error + self.previous_load_control_error)
        self.set_motor_angle(output)
        self.previous_load_control_output = output
        self.previous_load_control_error = current_error

    def loosen_if_overloaded(self, load):
        '''
        Takes the given load and checks against threshold, loosen motor if over
        '''
        if abs(load) > self.OVERLOAD_THRESHOLD:
            rospy.logwarn("Motor %s overloaded at %f, loosening" % (self.namespace, load))
            self.loosen()

    def tighten(self, tighten_angle=0.05):
        '''
        Takes the given angle offset in radians and tightens the motor
        '''
        if self.MOTOR_TO_JOINT_INVERTED:
            tighten_angle *= -1
        self.set_raw_motor_angle(self.motor_msg.raw_angle + tighten_angle)

    def loosen(self, loosen_angle=0.05):
        '''
        Takes the given angle offset in radians and loosens the motor
        '''
        if self.MOTOR_TO_JOINT_INVERTED:
            loosen_angle *= -1
        self.set_raw_motor_angle(self.motor_msg.raw_angle - loosen_angle)

    def receive_state_cb(self, data):
        # Calculate joint angle from motor angle
        if self.MOTOR_TO_JOINT_INVERTED:
            joint_angle = self.zero_point - data.current_pos
        else:
            joint_angle = data.current_pos - self.zero_point
        self.motor_msg.joint_angle = joint_angle / self.MOTOR_TO_JOINT_GEAR_RATIO
        self.motor_msg.raw_angle = data.current_pos
        self.motor_msg.velocity = data.velocity
        self.handle_motor_load(data.load)
        self.motor_msg.temperature = data.motor_temps[0]

    def handle_motor_load(self, load):
        load_filter = 0.25  # Rolling filter of noisy data
        if not self.MOTOR_TO_JOINT_INVERTED:
            load *= -1
        self.motor_msg.load = load_filter * load + (1 - load_filter) * self.motor_msg.load
        if self.in_control_torque_mode:
            self.control_torque(self.motor_msg.load)
        self.loosen_if_overloaded(self.motor_msg.load)
