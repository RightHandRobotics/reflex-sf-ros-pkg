import unittest

import mock

import reflex_sf_hand


class TestReflexSFHand(unittest.TestCase):
    @mock.patch('rospy.Subscriber')
    @mock.patch('motor.Motor')
    @mock.patch('rospy.loginfo')
    @mock.patch('rospy.init_node')
    def setUp(self, init_mock, loginfo_mock, motor_mock, sub_mock):
        self.rh = reflex_sf_hand.ReflexSFHand()

    @mock.patch('reflex_sf_hand.ReflexSFHand.set_angles')
    @mock.patch('reflex_sf_hand.ReflexSFHand.set_speeds')
    def test_receive_cmd_cb(self, set_speeds_mock, set_angles_mock):
        data = mock.MagicMock()
        data.velocity = 5.0
        data.pose = 14.0
        self.rh.receive_cmd_cb(data)
        set_speeds_mock.called_once_with(5.0)
        set_angles_mock.called_once_with(14.0)

    @mock.patch('reflex_sf_hand.ReflexSFHand.set_angles')
    @mock.patch('reflex_sf_hand.ReflexSFHand.reset_speeds')
    def test_receive_angle_cmd_cb(self, reset_speeds_mock, set_angles_mock):
        data = 12345
        self.rh.receive_angle_cmd_cb(data)
        reset_speeds_mock.called_once_with()
        set_angles_mock.called_once_with(12345)

    @mock.patch('reflex_sf_hand.ReflexSFHand.set_velocities')
    def test_receive_vel_cmd_cb(self, vel_mock):
        data = 12345
        self.rh.receive_vel_cmd_cb(data)
        vel_mock.called_once_with(12345)

    def test_set_angles(self):
        pose = mock.MagicMock()
        pose.f1 = 1.1
        pose.f2 = 2.2
        pose.f3 = 3.3
        pose.preshape = 4.4
        self.rh.set_angles(pose)
        self.rh.motors['/reflex_sf_f1'].set_motor_angle.called_once_with(1.1)
        self.rh.motors['/reflex_sf_f2'].set_motor_angle.called_once_with(2.2)
        self.rh.motors['/reflex_sf_f3'].set_motor_angle.called_once_with(3.3)
        self.rh.motors['/reflex_sf_preshape'].set_motor_angle.called_once_with(4.4)

    def test_set_velocities(self):
        velocity = mock.MagicMock()
        velocity.f1 = 1.1
        velocity.f2 = 2.2
        velocity.f3 = 3.3
        velocity.preshape = 4.4
        self.rh.set_velocities(velocity)
        self.rh.motors['/reflex_sf_f1'].set_motor_velocity.called_once_with(1.1)
        self.rh.motors['/reflex_sf_f2'].set_motor_velocity.called_once_with(2.2)
        self.rh.motors['/reflex_sf_f3'].set_motor_velocity.called_once_with(3.3)
        self.rh.motors['/reflex_sf_preshape'].set_motor_velocity.called_once_with(4.4)

    def test_set_angles(self):
        speed = mock.MagicMock()
        speed.f1 = 1.1
        speed.f2 = 2.2
        speed.f3 = 3.3
        speed.preshape = 4.4
        self.rh.set_speeds(speed)
        self.rh.motors['/reflex_sf_f1'].reset_motor_speed.called_once_with(1.1)
        self.rh.motors['/reflex_sf_f2'].set_motor_speed.called_once_with(2.2)
        self.rh.motors['/reflex_sf_f3'].set_motor_speed.called_once_with(3.3)
        self.rh.motors['/reflex_sf_preshape'].set_motor_speed.called_once_with(4.4)

    def test_reset_speeds(self):
        self.rh.reset_speeds()
        self.rh.motors['/reflex_sf_f1'].reset_motor_speed.called_once_with()
        self.rh.motors['/reflex_sf_f2'].reset_motor_speed.called_once_with()
        self.rh.motors['/reflex_sf_f3'].reset_motor_speed.called_once_with()
        self.rh.motors['/reflex_sf_preshape'].reset_motor_speed.called_once_with()

    def test_disable_torque(self):
        self.rh.disable_torque()
        self.rh.motors['/reflex_sf_f1'].disable_torque.called_once_with()
        self.rh.motors['/reflex_sf_f2'].disable_torque.called_once_with()
        self.rh.motors['/reflex_sf_f3'].disable_torque.called_once_with()
        self.rh.motors['/reflex_sf_preshape'].disable_torque.called_once_with()

    def test_enable_torque(self):
        self.rh.enable_torque()
        self.rh.motors['/reflex_sf_f1'].enable_torque.called_once_with()
        self.rh.motors['/reflex_sf_f2'].enable_torque.called_once_with()
        self.rh.motors['/reflex_sf_f3'].enable_torque.called_once_with()
        self.rh.motors['/reflex_sf_preshape'].enable_torque.called_once_with()

    @mock.patch('reflex_sf_hand.ReflexSFHand.write_zero_point_data_to_file')
    def test_zero_current_pose(self, write_mock):
        self.rh.zero_current_pose()
        self.rh.motors['/reflex_sf_f1'].get_current_raw_motor_angle.called_once_with()
        self.rh.motors['/reflex_sf_f2'].get_current_raw_motor_angle.called_once_with()
        self.rh.motors['/reflex_sf_f3'].get_current_raw_motor_angle.called_once_with()
        self.rh.motors['/reflex_sf_preshape'].get_current_raw_motor_angle.called_once_with()
        self.assertTrue(write_mock.called)


if __name__ == '__main__':
    unittest.main()
