#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function
import unittest
import time

from rosconversions import (get_message_class,
                            fill_ros_message)

class TestROSMsgTest(unittest.TestCase):
    def setUp(self):
        self.startTime = time.time()

    def tearDown(self):
        t = time.time() - self.startTime
        print("%s: %.3f" % (self.id(), t))

    def test_get_message_class(self):
        from sensor_msgs.msg import Image
        msg_cls = get_message_class('sensor_msgs/Image')
        msg = msg_cls()
        self.assertIsInstance(msg, Image)

    def test_fill_message(self):
        from std_msgs.msg import String
        d = {
            'data': "Filled"
        }
        msg = String()
        fill_ros_message(msg, d)
        self.assertIsInstance(msg, String, msg=None)
        self.assertEqual(d['data'], msg.data)

if __name__ == '__main__':
    unittest.main(verbosity=2)
