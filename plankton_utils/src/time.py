import rclpy

def time_in_float_sec(time: rclpy.time.Time):
    f_time = time.seconds_nanoseconds[0] + time.seconds_nanoseconds[1] / 1e9
    return f_time