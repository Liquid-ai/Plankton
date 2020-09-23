# Copyright (c) 2020 The Plankton Authors.
# All rights reserved.
#
import unittest

from rclpy.time import Time

from plankton_utils.time import time_in_float_sec


class TestClient(unittest.TestCase):

    def test_time_to_float(self):
        EPS = 1e-4

        t = Time()
        self.assertAlmostEqual(time_in_float_sec(t), 0.0, delta=EPS)

        t = Time(seconds=1, nanoseconds=0.5*1e9)
        self.assertAlmostEqual(time_in_float_sec(t), 1.5, delta=EPS)

        t = Time(seconds=1, nanoseconds=1e9)
        self.assertAlmostEqual(time_in_float_sec(t), 2.0, delta=EPS)

        t = Time(seconds=1, nanoseconds=15*1e9)
        self.assertAlmostEqual(time_in_float_sec(t), 16, delta=EPS)

        t = Time(seconds=20, nanoseconds=15*1e9)
        self.assertAlmostEqual(time_in_float_sec(t), 35, delta=EPS)


    # def test_wait_for_service_5sec(self):
    #     cli = self.node.create_client(GetParameters, 'get/parameters')
    #     try:
    #         start = time.monotonic()
    #         self.assertFalse(cli.wait_for_service(timeout_sec=5.0))
    #         end = time.monotonic()
    #         self.assertGreater(5.0, end - start - TIME_FUDGE)
    #         self.assertLess(5.0, end - start + TIME_FUDGE)
    #     finally:
    #         self.node.destroy_client(cli)

    # def test_wait_for_service_nowait(self):
    #     cli = self.node.create_client(GetParameters, 'get/parameters')
    #     try:
    #         start = time.monotonic()
    #         self.assertFalse(cli.wait_for_service(timeout_sec=0))
    #         end = time.monotonic()
    #         self.assertGreater(0, end - start - TIME_FUDGE)
    #         self.assertLess(0, end - start + TIME_FUDGE)
    #     finally:
    #         self.node.destroy_client(cli)

    # def test_wait_for_service_exists(self):
    #     cli = self.node.create_client(GetParameters, 'test_wfs_exists')
    #     srv = self.node.create_service(GetParameters, 'test_wfs_exists', lambda request: None)
    #     try:
    #         start = time.monotonic()
    #         self.assertTrue(cli.wait_for_service(timeout_sec=1.0))
    #         end = time.monotonic()
    #         self.assertGreater(0, end - start - TIME_FUDGE)
    #         self.assertLess(0, end - start + TIME_FUDGE)
    #     finally:
    #         self.node.destroy_client(cli)
    #         self.node.destroy_service(srv)

    # def test_concurrent_calls_to_service(self):
    #     cli = self.node.create_client(GetParameters, 'get/parameters')
    #     srv = self.node.create_service(
    #         GetParameters, 'get/parameters',
    #         lambda request, response: response)
    #     try:
    #         self.assertTrue(cli.wait_for_service(timeout_sec=20))
    #         future1 = cli.call_async(GetParameters.Request())
    #         future2 = cli.call_async(GetParameters.Request())
    #         executor = rclpy.executors.SingleThreadedExecutor(context=self.context)
    #         rclpy.spin_until_future_complete(self.node, future1, executor=executor)
    #         rclpy.spin_until_future_complete(self.node, future2, executor=executor)
    #         self.assertTrue(future1.result() is not None)
    #         self.assertTrue(future2.result() is not None)
    #     finally:
    #         self.node.destroy_client(cli)
    #         self.node.destroy_service(srv)

    # # https://github.com/ros2/rmw_connext/issues/405
    # @unittest.skipIf(
    #     get_rmw_implementation_identifier() == 'rmw_connext_cpp',
    #     reason='Source timestamp not implemented for Connext')
    # def test_service_timestamps(self):
    #     cli = self.node.create_client(GetParameters, 'get/parameters')
    #     srv = self.node.create_service(
    #         GetParameters, 'get/parameters',
    #         lambda request, response: response)
    #     try:
    #         self.assertTrue(cli.wait_for_service(timeout_sec=20))
    #         cli.call_async(GetParameters.Request())
    #         cycle_count = 0
    #         while cycle_count < 5:
    #             with srv.handle as capsule:
    #                 result = _rclpy.rclpy_take_request(capsule, srv.srv_type.Request)
    #             if result is not None:
    #                 request, header = result
    #                 source_timestamp = _rclpy.rclpy_service_info_get_source_timestamp(header)
    #                 self.assertNotEqual(0, source_timestamp)
    #                 return
    #             else:
    #                 time.sleep(0.1)
    #         self.fail('Did not get a request in time')
    #     finally:
    #         self.node.destroy_client(cli)
    #         self.node.destroy_service(srv)

    # def test_different_type_raises(self):
    #     cli = self.node.create_client(GetParameters, 'get/parameters')
    #     srv = self.node.create_service(
    #         GetParameters, 'get/parameters',
    #         lambda request, response: 'different response type')
    #     try:
    #         with self.assertRaises(TypeError):
    #             cli.call('different request type')
    #         with self.assertRaises(TypeError):
    #             cli.call_async('different request type')
    #         self.assertTrue(cli.wait_for_service(timeout_sec=20))
    #         future = cli.call_async(GetParameters.Request())
    #         executor = rclpy.executors.SingleThreadedExecutor(context=self.context)
    #         with self.assertRaises(TypeError):
    #             rclpy.spin_until_future_complete(self.node, future, executor=executor)
    #     finally:
    #         self.node.destroy_client(cli)
    #         self.node.destroy_service(srv)

#==============================================================================
if __name__ == '__main__':
    unittest.main()