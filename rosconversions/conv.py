#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Highly inspired from: https://github.com/uts-magic-lab/rosduct
"""

from __future__ import print_function

import roslib.message
import rospy
import re
import base64

ROS_DATATYPE_MAP = {
    'bool': ['bool'],
    'int': ['int8', 'byte', 'uint8', 'char',
            'int16', 'uint16', 'int32', 'uint32',
            'int64', 'uint64', 'float32', 'float64'],
    'float': ['float32', 'float64'],
    'str': ['string'],
    'unicode': ['string'],
    'long': ['uint64']
}

PRIMITIVE_TYPES = [bool, int, float]
STRING_TYPES = [str]
LIST_TYPES = [list, tuple]

ROS_TIME_TYPES = ['time', 'duration']
ROS_PRIMITIVE_TYPES = ['bool', 'byte', 'char', 'int8', 'uint8', 'int16',
                       'uint16', 'int32', 'uint32', 'int64', 'uint64',
                       'float32', 'float64', 'string']
ROS_HEADER_TYPES = ['Header', 'std_msgs/Header', 'roslib/Header']
ROS_BINARY_TYPES_REGEX = re.compile(r'(uint8|char)\[[^\]]*\]')
ROS_BINARY_TYPES_REGEX_2 = r'(uint8|char)\[[^\]]*\]'
LIST_BRACKETS = re.compile(r'\[[^\]]*\]')


def _to_ros_type(_type, _value):
    if _is_ros_binary_type(_type):
        #  print("Type {0} is binary type".format(_type))
        _value = _to_ros_binary(_type, _value)
    elif _type in ROS_TIME_TYPES:
        #  print("Type {0} is time type".format(_type))
        _value = _to_ros_time(_type, _value)
    elif _type in ROS_PRIMITIVE_TYPES:
        #  print("Type {0} is primitive type".format(_type))
        _value = _to_ros_primitive(_type, _value)
    elif _is_type_an_array(_type):
        #  print("Type {0} is array type".format(_type))
        _value = _convert_to_ros_array(_type, _value)
    else:
        _value = dict_to_ros(_type, _value)
    return _value


def _to_ros_binary(_type, _value):
    #  print("To ROS Binary: {}, {}".format(_type, type(_value)))
    # Convert to utf8 string if data format is unicode. ROS does not accept unicodes.
    if isinstance(_value, unicode):
        _value = _value.encode('utf8')
    binary_value_as_string = _value
    if type(_value) in STRING_TYPES:
        binary_value_as_string = base64.standard_b64decode(_value)
    else:
        binary_value_as_string = str(bytearray(_value))
    return binary_value_as_string


def _to_ros_time(_type, _value):
    time = None

    if _type == 'time' and _value == 'now':
        time = rospy.get_rostime()
    else:
        if _type == 'time':
            time = rospy.rostime.Time()
        elif _type == 'duration':
            time = rospy.rostime.Duration()
        if 'secs' in _value:
            setattr(time, 'secs', _value['secs'])
        if 'nsecs' in _value:
            setattr(time, 'nsecs', _value['nsecs'])
    return time


def _to_ros_primitive(_type, _value):
    if _type == "string" and isinstance(_value, unicode):
        _value = _value.encode('utf8')
    elif _type in ["float32", "float64"]:
        if _value == None:
            _value = float('Inf')
        #  if math.isnan(_value) or math.isinf(_value):
            #  print("IS Inf or NaN")
    return _value


def _convert_to_ros_array(_type, list_value):
    list_type = LIST_BRACKETS.sub('', _type)
    return [_to_ros_type(list_type, value) for value in list_value]


def _from_ros_type(_type, _value):
    if _is_ros_binary_type(_type):
        #  print("Data is ROS binary type")
        _value = _from_ros_binary(_value)
    elif _type in ROS_TIME_TYPES:
        _value = _from_ros_time(_value)
    elif _type in ROS_PRIMITIVE_TYPES:
        #  print("Data is ROS primitive type")
        _value = _value
    elif _is_type_an_array(_type):
        #  print("Data is Array")
        _value = _from_ros_array(_type, _value)
    else:
        _value = ros_to_dict(_value)
    return _value


def _is_ros_binary_type(_type):
    """Check if the field is a binary array one, fixed size or not

    Args:
        _type (str): The ROS message field type.
        _val (str|int|char|...): The ROS message field value.
    """
    c = re.search(ROS_BINARY_TYPES_REGEX, _type) is not None
    #  print("IS ROS BINARY: {}, {}".format(_type, c))
    return c


def _from_ros_binary(_value):
    """ROS encodes buffers (uint8[]) into base64 strings.

    Args:
        _type (str): Data to serialize.
    """
    _value = base64.standard_b64encode(_value)
    return _value


def _from_ros_time(_value):
    _value = {
        'secs': _value.secs,
        'nsecs': _value.nsecs
    }
    return _value


def _from_ros_primitive(_type, _value):
    return _value


def _from_ros_array(_type, _value):
    list_type = LIST_BRACKETS.sub('', _type)
    return [_from_ros_type(list_type, value) for value in _value]


def _get_message_fields(message):
    """ From here: http://wiki.ros.org/msg -> 4. Client Library Support

    In Python, the generated Python message file (e.g. std_msgs.msg.String) provides
    nearly all the information you might want about a .msg file. You can examine the
    __slots__ and _slot_types and other fields to introspect information about messages.
    """
    return zip(message.__slots__, message._slot_types)


def _is_type_an_array(_type):
    return LIST_BRACKETS.search(_type) is not None


def dict_to_ros(message_type, dictionary):
    """Take in the message type and a Python dictionary and returns
    a ROS message.

    Args:
        message_type (str): The ROS Message type, e.g. ``std_msgs/String``
        dictionary (dict): The dictionary to transform to ROS Message

    Example:
        message_type = "std_msgs/String"
        dict_message = { "data": "Hello, Robot" }
        ros_message = dict_to_ros(message_type, dict_message)
    """
    message_class = roslib.message.get_message_class(message_type)
    message = message_class()
    message_fields = dict(_get_message_fields(message))

    for field_name, _value in dictionary.items():
        if field_name in message_fields:
            _type = message_fields[field_name]
            _value = _to_ros_type(_type, _value)
            setattr(message, field_name, _value)
        else:
            err = 'ROS message type "{0}" has no field named "{1}"'\
                .format(message_type, field_name)
            raise ValueError(err)
    return message


def ros_to_dict(message):
    """Take in a ROS message and returns a Python dictionary.

    Example:
        ros_message = std_msgs.msg.String(data="Hello, Robot")
        dict_message = ros_to_dict(ros_message)
    """
    dictionary = {}
    message_fields = _get_message_fields(message)
    for field_name, _type in message_fields:
        _value = getattr(message, field_name)
        dictionary[field_name] = _from_ros_type(_type, _value)
    return dictionary


def get_message_class(message_type):
    return roslib.message.get_message_class(message_type)


if __name__ == "__main__":
    from sensor_msgs.msg import Image
    message = Image()

    print("------------------<sensor_msgs/Image>-------------------")
    from PIL import Image
    im = Image.open('Lenna.png')
    width, height = im.size

    message.height = height
    message.width = width
    message.step = width
    message.data = str(list(im.getdata()))

    print("From sensor_msgs/Image to dict...")
    _dict = ros_to_dict(message)
    print("From dict to sensor_msgs/Image...")
    msg = dict_to_ros("sensor_msgs/Image", _dict)
    print("-------------------------------------------------------")
