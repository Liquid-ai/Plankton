import rclpy
from rclpy.clock import ClockType

def time_in_float_sec(time: rclpy.time.Time):
    sec_nano = time.seconds_nanoseconds()
    time.from_msg()
    f_time = sec_nano[0] + sec_nano[1] / 1e9
    return f_time

#==============================================================================
def time_in_float_sec_from_msg(time_msg, clock_type=ClockType.ROS_TIME):
    time_object = rclpy.time.Time.from_msg(time_msg, clock_type)
    return time_in_float_sec(time_object)