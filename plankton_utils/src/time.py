import rclpy

def time_in_float_sec(time: rclpy.time.Time):
    sec_nano = time.seconds_nanoseconds()
    f_time = sec_nano[0] + sec_nano[1] / 1e9
    return f_time