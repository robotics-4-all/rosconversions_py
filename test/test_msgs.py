#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function
import unittest
import time

from rosconversions import dict_to_ros, ros_to_dict, get_message_class

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

if __name__ == '__main__':
    unittest.main(verbosity=2)
