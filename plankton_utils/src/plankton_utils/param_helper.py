# Copyright (c) 2020 The Plankton Authors.
# All rights reserved.
#
import rclpy


def __merge_dicts(a, b):
    """
    merges b into a and return merged result

    NOTE: tuples and arbitrary objects are not handled as it is totally ambiguous what should happen
    """
    key = None
    try:
        if a is None or isinstance(a, str) or isinstance(a, int) or isinstance(a, float):
            # border case for first run or if a is a primitive
            a = b
        elif isinstance(a, list):
            # lists can be only appended
            if isinstance(b, list):
                # merge lists
                a.extend(b)
            else:
                # append to list
                a.append(b)
        elif isinstance(a, dict):
            # dicts must be merged
            if isinstance(b, dict):
                for key in b:
                    if key in a:
                        a[key] = __merge_dicts(a[key], b[key])
                    else:
                        a[key] = b[key]
            else:
                raise RuntimeError('Cannot merge non-dict "%s" into dict "%s"' % (b, a))
        else:
            raise RuntimeError('NOT IMPLEMENTED "%s" into "%s"' % (b, a))
    except TypeError as e:
        raise RuntimeError('TypeError "%s" in key "%s" when merging "%s" into "%s"' % (e, key, b, a))
    return a


# ==============================================================================  
def parse_nested_params_to_dict(this_list, separator: str = ".", unpack_value: bool = False):
    """
    From a dictionary of namespaced ROS 2 parameters, e.g.:  
    {ns1.ns2.param1:value1, ns1.s2.param2:value2, ns3.param3:value3}, returns a 
    dictionary of n dictionaries, e.g.: 
    d['ns1']['ns2]['param1'], d['ns1']['ns2]['param2'], d['ns3']['param3']

    :param this_list: list to parse
    :param separator: separator used in the list
    :param unpack_value: False to create dictionary of Parameters, True to get the final values

    :return The created dictionary
    """
      
    parameters_with_prefix = {}

    for parameter_name, param_value in this_list.items():
        dotFound = True

        dict_ = {}
        key_list = []
        while dotFound:
            dotFound = False
            key = ""
            index  = str(parameter_name).find(separator)
            if index != -1:
                dotFound = True
                key = parameter_name[:index]
                key_list.insert(0, key)

                # dict_.update({key: {}})
                parameter_name = parameter_name[index + 1:]
            else:
                key_list.insert(0, parameter_name)

        # Create the associated dictionary
        for i, key in enumerate(key_list):
            if i == 0:
                dict_.update({key: param_value if not unpack_value else param_value.value})
            else:
                dict_ = ({key: dict_})
        # Initialize the final global dictionary or merge
        if len(parameters_with_prefix.keys()) == 0:
            parameters_with_prefix = dict_
        else:
            parameters_with_prefix = __merge_dicts(parameters_with_prefix, dict_)
    return parameters_with_prefix


# =============================================================================
def get_parameter_or_helper(node, name, default_value):
    default_param = rclpy.Parameter(
            name, rclpy.Parameter.Type.from_parameter_value(default_value), default_value)
    return node.get_parameter_or(name, default_param)


# ==============================================================================    
# TODO Complete function
def remove_prefix_nested_params(this_list, separator: str = "."):
    """
    From a dictionary of namespaced ROS 2 parameters, e.g.:  
    {ns1.ns2.param1:value1, ns1.s2.param2:value2, ns3.param3:value3}, removes the 
    first prefix for each key and returns the dictionary
    """
    return
