import unittest

import mock

import motor


class TestMotor(unittest.TestCase):
    @mock.patch('rospy.get_param')
    @mock.patch('rospy.ServiceProxy')
    @mock.patch('rospy.Publisher')
    @mock.patch('rospy.Subscriber')
    def setUp(self, param_mock, proxy_mock, pub_mock, sub_mock):
        self.name = '/reflex_sf_f1'
        self.motor = motor.Motor(self.name)

    @mock.patch('rospy.set_param')
    def test_set_local_motor_zero_point(self, set_param_mock):
        self.motor.current_raw_position = 12345
        set_param_mock.called_once_with(self.name + '/zero_point', 12345)

    @mock.patch('motor.Motor.correct_motor_offset')
    def test_check_motor_angle_command(self, correct_mock):
        self.motor.zero_point = 0.0
        self.motor.MAX_MOTOR_TRAVEL = 5.0
        self.motor.MOTOR_TO_JOINT_INVERTED = False
        self.motor.MOTOR_TO_JOINT_GEAR_RATIO = 1.0
        correct_mock.return_value = 50.0
        self.assertAlmostEqual(self.motor.check_motor_angle_command(0.0), 5.0)

        correct_mock.return_value = -50.0
        self.assertAlmostEqual(self.motor.check_motor_angle_command(0.0), 0.0)

        correct_mock.return_value = 2.5
        self.assertAlmostEqual(self.motor.check_motor_angle_command(0.0), 2.5)

        self.motor.zero_point = 10.0
        self.motor.MOTOR_TO_JOINT_INVERTED = True
        correct_mock.return_value = 50.0
        self.assertAlmostEqual(self.motor.check_motor_angle_command(0.0), 10.0)

        correct_mock.return_value = -50.0
        self.assertAlmostEqual(self.motor.check_motor_angle_command(0.0), 5.0)

        correct_mock.return_value = 7.5
        self.assertAlmostEqual(self.motor.check_motor_angle_command(0.0), 7.5)

    @mock.patch('motor.Motor.check_motor_angle_command')
    @mock.patch('motor.Motor.set_motor_speed')
    def test_set_motor_velocity(self, check_mock, set_speed_mock):
        self.motor.set_motor_velocity(4.11)
        set_speed_mock.called_once_with(4.11)
        check_mock.called_once_with(self.motor.MAX_MOTOR_TRAVEL)

        self.motor.set_motor_velocity(-3.98)
        set_speed_mock.called_once_with(-3.98)
        check_mock.called_once_with(0.0)

    @mock.patch('motor.Motor.check_motor_speed_command')
    def test_set_motor_speed(self, check_mock):
        check_mock.return_value = 11.6
        self.motor.set_speed_service = mock.MagicMock()
        self.motor.set_motor_speed(6.1)
        check_mock.called_once_with(6.1)
        self.motor.set_speed_service.called_once_with(11.6)

    def test_check_motor_speed_command(self):
        self.motor.MAX_MOTOR_SPEED = 5.0
        self.assertAlmostEqual(self.motor.check_motor_speed_command(10.0), 5.0)
        self.assertAlmostEqual(self.motor.check_motor_speed_command(-10.0), 5.0)
        self.assertAlmostEqual(self.motor.check_motor_speed_command(4.99), 4.99)
        self.assertAlmostEqual(self.motor.check_motor_speed_command(-3.5), 3.5)

    def test_reset_motor_speed(self):
        self.motor.DEFAULT_MOTOR_SPEED = 10.0
        self.motor.set_speed_service = mock.MagicMock()
        self.motor.reset_motor_speed()
        self.motor.set_speed_service.called_once_with(10.0)

    def test_correct_motor_offset(self):
        self.motor.zero_point = 15.5
        self.motor.MOTOR_TO_JOINT_INVERTED = False
        self.assertAlmostEqual(self.motor.correct_motor_offset(4.5), 20.0)

        self.motor.MOTOR_TO_JOINT_INVERTED = True
        self.assertAlmostEqual(self.motor.correct_motor_offset(4.5), 11.0)

    @mock.patch('motor.Motor.loosen')
    @mock.patch('rospy.logwarn')
    def test_loosen_if_overloaded(self, warn_mock, loose_mock):
        self.motor.OVERLOAD_THRESHOLD = 0.5
        self.motor.loosen_if_overloaded(0.25)
        self.assertFalse(loose_mock.called)
        self.motor.loosen_if_overloaded(-0.25)
        self.assertFalse(loose_mock.called)

        self.motor.loosen_if_overloaded(0.75)
        loose_mock.called_once_with()
        self.motor.loosen_if_overloaded(-0.75)
        loose_mock.called_once_with()

    @mock.patch('motor.Motor.set_raw_motor_angle')
    def test_tighten_loosen(self, raw_mock):
        self.motor.MOTOR_TO_JOINT_INVERTED = False
        self.motor.motor_msg.raw_angle = 15.05
        self.motor.tighten()
        raw_mock.called_once_with(15.1)
        self.motor.tighten(0.95)
        raw_mock.called_once_with(16.0)
        self.motor.loosen()
        raw_mock.called_once_with(15.0)
        self.motor.loosen(0.95)
        raw_mock.called_once_with(14.1)

        self.motor.MOTOR_TO_JOINT_INVERTED = True
        self.motor.tighten()
        raw_mock.called_once_with(15.0)
        self.motor.tighten(0.95)
        raw_mock.called_once_with(14.1)
        self.motor.loosen()
        raw_mock.called_once_with(15.1)
        self.motor.loosen(0.95)
        raw_mock.called_once_with(16.0)

    @mock.patch('motor.Motor.loosen_if_overloaded')
    def test_receive_state_cb(self, loose_mock):
        data = mock.MagicMock()
        data.current_pos = 20.0
        data.load = 10.0
        self.motor.zero_point = 12.5
        self.motor.motor_msg.load = 22.0
        self.motor.MOTOR_TO_JOINT_INVERTED = False
        self.motor.MOTOR_TO_JOINT_GEAR_RATIO = 2.0

        self.motor.receive_state_cb(data)
        self.assertAlmostEqual(self.motor.motor_msg.raw_angle, 20.0)
        self.assertAlmostEqual(self.motor.motor_msg.load, 20.8)
        self.assertAlmostEqual(self.motor.motor_msg.joint_angle, 3.75)
        loose_mock.called_once_with(20.8)

        self.motor.MOTOR_TO_JOINT_INVERTED = True
        self.motor.receive_state_cb(data)
        self.assertAlmostEqual(self.motor.motor_msg.joint_angle, -3.75)


if __name__ == '__main__':
    unittest.main()
