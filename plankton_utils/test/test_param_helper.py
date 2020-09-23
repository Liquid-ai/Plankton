# Copyright (c) 2020 The Plankton Authors.
# All rights reserved.
#
#
import unittest

from rclpy.time import Time

from plankton_utils.param_helper import *


class TestClient(unittest.TestCase):

    def test_to_dict(self):
        SEP = '.' 
        list_1 = {'a':1, 'b':'hello'}
        res = parse_nested_params_to_dict(list_1, SEP)
        self.assertDictEqual(list_1, res)

        #
        list_1 = {'a.b':1, 'a.c':3.0}
        res = parse_nested_params_to_dict(list_1, SEP)
        self.assertEqual(list_1['a.b'], res['a']['b'])
        self.assertEqual(list_1['a.c'], res['a']['c'])

        #
        list_1 = {'a.b':1, 'b.a':3}
        res = parse_nested_params_to_dict(list_1, SEP)
        self.assertEqual(list_1['a.b'], res['a']['b'])
        self.assertEqual(list_1['b.a'], res['b']['a'])

        #
        list_1 = {'a.b':1, 'a.c':3.0, 'b':5.0, 'c.a.b':'ok'}
        res = parse_nested_params_to_dict(list_1, SEP)
        self.assertEqual(list_1['a.b'], res['a']['b'])
        self.assertEqual(list_1['a.c'], res['a']['c'])
        self.assertEqual(list_1['b'], res['b'])
        self.assertEqual(list_1['c.a.b'], res['c']['a']['b'])

        #
        list_1 = {'a.a':1, 'b.b':3}
        res = parse_nested_params_to_dict(list_1, SEP)
        self.assertEqual(list_1['a.a'], res['a']['a'])
        self.assertEqual(list_1['b.b'], res['b']['b'])

        #
        list_1 = {'a.b.c.d.e':1, 'a.b.c.a':3}
        res = parse_nested_params_to_dict(list_1, SEP)
        self.assertEqual(list_1['a.b.c.d.e'], res['a']['b']['c']['d']['e'])
        self.assertEqual(list_1['a.b.c.a'], res['a']['b']['c']['a'])


