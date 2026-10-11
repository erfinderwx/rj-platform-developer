#!/usr/bin/env python3
"""rostest: drive the simulated chassis through the ROS 1 driver and check odometry and state.
Luchs B also checks the LCI services (speed cap, e-stop latched by the chassis)."""
import sys
import unittest

import rospy
import rostest
from geometry_msgs.msg import Twist
from luchs_msgs.msg import ChassisState
from luchs_msgs.srv import SetSpeedLimit
from nav_msgs.msg import Odometry
from std_srvs.srv import SetBool

MODEL = sys.argv[1] if len(sys.argv) > 1 and sys.argv[1] in ('a', 'b') else 'b'


class DriveCheck(unittest.TestCase):
    def setUp(self):
        self.odom = None
        self.state = None
        rospy.Subscriber('/odom', Odometry, lambda m: setattr(self, 'odom', m))
        rospy.Subscriber('/luchs_driver/state', ChassisState, lambda m: setattr(self, 'state', m))
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    def drive(self, vx, wz, seconds):
        t = Twist()
        t.linear.x, t.angular.z = vx, wz
        end = rospy.Time.now() + rospy.Duration(seconds)
        r = rospy.Rate(20)
        while rospy.Time.now() < end and not rospy.is_shutdown():
            self.pub.publish(t)
            r.sleep()

    def test_drive(self):
        rospy.sleep(2.0)
        self.drive(0.8, 0.2, 3.0)
        self.assertIsNotNone(self.odom)
        self.assertAlmostEqual(self.odom.twist.twist.linear.x, 0.8, delta=0.15)
        self.assertGreater(self.odom.pose.pose.position.x, 0.5)
        self.assertTrue(self.state.feedback_ok)
        if MODEL == 'b':
            self.assertTrue(self.state.lci_ok)
            self.assertEqual(self.state.model_text, 'Luchs B')
            self.assertFalse(self.state.model_mismatch)
            rospy.ServiceProxy('/luchs_driver/set_speed_limit', SetSpeedLimit)(0.5)
            self.drive(0.8, 0.0, 2.5)
            self.assertAlmostEqual(self.odom.twist.twist.linear.x, 0.5, delta=0.1)
            self.assertEqual(self.state.limit_reason_text, 'host speed limit')
            rospy.ServiceProxy('/luchs_driver/estop', SetBool)(True)
            self.drive(0.4, 0.0, 2.0)
            self.assertAlmostEqual(self.odom.twist.twist.linear.x, 0.0, delta=0.05)
            self.assertIn('host request', self.state.estop_source_text)


if __name__ == '__main__':
    rospy.init_node('drive_check')
    rostest.rosrun('luchs_driver', 'drive_check_' + MODEL, DriveCheck, sys.argv)
