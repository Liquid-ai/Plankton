#!/usr/bin/env python3
from rclpy.time import Time

def time_in_float_sec(time: Time):
    f_time = time.seconds_nanoseconds[0] + time.seconds_nanoseconds[1] / 1e9
    return f_time