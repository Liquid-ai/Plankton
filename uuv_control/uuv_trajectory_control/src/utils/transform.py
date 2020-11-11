# Copyright (c) 2020 The Plankton Authors.
# All rights reserved.
#
import os
import threading
import tf2_py
import tf2_ros    
import time
import rclpy

def get_world_ned_to_enu(sim_time_param):  
    if not rclpy.ok():
        raise RuntimeError('Error: this function requires that rclpy '
                           'is running')

    tf_trans_ned_to_enu = None
    done = False
    node = rclpy.create_node('tf_get_ned_to_enu_%d' % os.getpid(), parameter_overrides=[sim_time_param])

    tf_buffer = tf2_ros.Buffer(node=node)
    listener = tf2_ros.TransformListener(tf_buffer, node)

    # time.sleep(5)

    future = tf_buffer.wait_for_transform_async('world', 'world_ned', rclpy.time.Time())

    rclpy.spin_until_future_complete(node, future, timeout_sec=5)
    try:
        tf_trans_ned_to_enu = tf_buffer.lookup_transform(
                        'world', 'world_ned', rclpy.time.Time(),
                        rclpy.time.Duration(seconds=10))
    except tf2_ros.LookupException:
        node.get_logger().warn('Transform world ned to enu not found. Skipping')
        pass

    node.destroy_node()
    return tf_trans_ned_to_enu

    