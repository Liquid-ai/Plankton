# Copyright (c) 2020 The Plankton Authors.
# All rights reserved.
#
import rclpy.time
from rclpy.clock import ClockType


def time_in_float_sec(time: rclpy.time.Time):
    """
    From a Time object, returns the associated time in float seconds
    """
    sec_nano = time.seconds_nanoseconds()
    f_time = sec_nano[0] + sec_nano[1] / 1e9
    return f_time


# =============================================================================
def time_in_float_sec_from_msg(time_msg, clock_type=ClockType.ROS_TIME):
    """From a Time message, returns the associated time in float seconds."""
    time_object = rclpy.time.Time.from_msg(time_msg, clock_type)
    return time_in_float_sec(time_object)


# =============================================================================
def float_sec_to_int_sec_nano(float_sec):
    """
    From a floating value in seconds, returns a tuple of integer seconds and 
    nanoseconds
    """
    secs = int(float_sec)
    nsecs = int((float_sec - secs) * 1e9)

    return (secs, nsecs)
