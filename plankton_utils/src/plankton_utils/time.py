# Copyright (c) 2020 The Plankton Authors.
# All rights reserved.
#
import rclpy.time
from rclpy.clock import ClockType
from rcl_interfaces.srv import GetParameters

import time


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
    

# =============================================================================
def __is_sim_time_subprocess(timeout_sec= 5, default_value=False, return_param=True):
    """
    Sends a request to the global sim time node and returns the value of the 
    use_sim_time parameter running a command.

    :param timeout_sec: timeout in seconds before returning the default value
    :param return_param: True to return a parameter, False to return the raw
    boolean
    :param default_value: default value to return

    :return A boolean or a parameter, both indicating if Plankton is using
    sim time
    """
    res = default_value
    starting_time = time.time()
    retry = True
    # Loop until the timeout has expired
    while retry:
        try:
            import subprocess
            from subprocess import TimeoutExpired

            output = subprocess.check_output(
                ['ros2 param get /plankton_global_sim_time use_sim_time'], 
                timeout=max(timeout_sec - (time.time() - starting_time), 0),
                shell=True
            )

            output = output.decode()
            res = True if 'True' in output else False
            retry = False

        except TimeoutExpired:
            print('Could not request for sim time. Defaulting to %s' % default_value)
            retry = False
            pass
        except subprocess.CalledProcessError:
            if time.time() > starting_time + timeout_sec:
                print('Sim time service not available. '
                'Defaulting to %s' % default_value)
                retry = False
            pass
        except Exception as e:
            print('Unexpected exception while requesting sim time (%s). '
                'Defaulting to %s' % (repr(e), default_value))
            retry = False
            pass


    if return_param:
        return rclpy.parameter.Parameter(
            'use_sim_time', 
            rclpy.Parameter.Type.BOOL, 
            res
        )
    else:
        return res


# =============================================================================
def __is_sim_time_node(timeout_sec=10, return_param=True, default_value=False):
    """
    Sends a request to the global sim time node and returns the value of the 
    use_sim_time parameter. If a 'no_global_sim_time' parameter has been
    specified to the node, the function returns the default value.

    :param timeout_sec: timeout in seconds before returning the default value
    :param return_param: True to return a parameter, False to return the raw
    boolean
    :param default_value: default value to return

    :return A boolean or a parameter, both indicating if Plankton is using
    sim time
    """
    try:
        def get_value(value):
            if return_param:
                return rclpy.parameter.Parameter(
                    'use_sim_time', 
                    rclpy.Parameter.Type.BOOL, 
                    value
                )
            else:
                return value
        
        import random
        import os
        import string
    
        node = None

        LENGTH = 5

        if not rclpy.ok():
            raise RuntimeError('rclpy has not been initialized. Initialize rclpy first')

        letter_pool = string.ascii_letters
        random_name = 'test_sim_time_' + ''.join(random.choice(letter_pool) for i in range(LENGTH))
        random_name += '_%d' % os.getpid()

        node = rclpy.create_node(random_name)

        if node.has_parameter('no_global_sim_time'):
            return get_value(default_value)

        start_time = time.time()

        sim_time_srv = node.create_client(GetParameters, '/plankton_global_sim_time/get_parameters')
        if not sim_time_srv.wait_for_service(timeout_sec=timeout_sec):
            node.get_logger().info('service %s not available' % sim_time_srv.srv_name)
            return get_value(default_value)

        # Compute the remaining time
        timeout_sec -= time.time() - start_time
        
        req = GetParameters.Request()
        req.names = ['use_sim_time']
        future = sim_time_srv.call_async(req)

        start_time = time.time()
        while not future.done():
            rclpy.spin_once(node)
            if time.time() - start_time > timeout_sec:
                return get_value(default_value)

        resp = future.result().values[0].bool_value

        return get_value(resp)

    except Exception as e:
        print('Caught exception: ' + repr(e))

    finally:
        if node is not None:
            node.destroy_node()
            node = None


# =============================================================================
def is_sim_time(
    timeout_sec=10, 
    return_param=True, 
    default_value=False, 
    use_subprocess=False
):
    """
    Sends a request to the global sim time node and returns the value of the 
    use_sim_time parameter. If not using subprocess to send the request and 
    if a 'no_global_sim_time' parameter has been specified to the node, the 
    function returns the default value.

    :param timeout_sec: timeout in seconds before returning the default value
    :param return_param: True to return a parameter, False to return the raw
    boolean
    :param default_value: default value to return
    :param use_subprocess: True to run a command with ros2 param arguments,
    False to create a node and use a service

    :return A boolean or a parameter, both indicating if Plankton is using
    sim time
    """
    if use_subprocess:
        return __is_sim_time_subprocess(timeout_sec=timeout_sec, 
                return_param=return_param, default_value=default_value)
    else:
        return __is_sim_time_node(timeout_sec=timeout_sec, 
                return_param=return_param, default_value=default_value)
