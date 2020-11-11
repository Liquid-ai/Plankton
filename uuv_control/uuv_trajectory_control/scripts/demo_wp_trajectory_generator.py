#!/usr/bin/env python3
# Copyright (c) 2020 The Plankton Authors.
# All rights reserved.
#
# This source code is derived from UUV Simulator
# (https://github.com/uuvsimulator/uuv_simulator)
# Copyright (c) 2016-2019 The UUV Simulator Authors
# licensed under the Apache license, Version 2.0
# cf. 3rd-party-licenses.txt file in the root directory of this source tree.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import numpy as np
import matplotlib.pyplot as plt
import uuv_trajectory_generator
from uuv_trajectory_generator.wp_trajectory_generator import WPTrajectoryGenerator
from uuv_waypoints.waypoint_set import WaypointSet
from uuv_waypoints.waypoint_set import Waypoint
import time
from geometry_msgs.msg import Point
from mpl_toolkits.mplot3d import Axes3D

"""
Demo file to demonstrate the waypoint interpolation method with generation of
velocity and acceleration profile using a constant rate.
"""

def run_generator(waypoint_set, interp_method):
    # Initialize the trajectory generator
    namespace = ''
    gen = WPTrajectoryGenerator(namespace, full_dof=False)
    gen.set_interpolation_method(interp_method)
    gen.init_waypoints(waypoint_set)

    dt = 0.05
    idx = 0
    pnts = list()
    avg_time = 0.0

    gen.set_start_time(0)
    for ti in np.arange(-2, gen.get_max_time(), dt):
        tic = time.clock()
        pnts.append(gen.interpolate(ti))
        toc = time.clock()
        avg_time += toc - tic
        idx += 1
    avg_time /= idx
    print('Average processing time [s] =', avg_time)
    fig = plt.figure()
    # Trajectory and heading 3D plot
    ax = fig.add_subplot(111, projection='3d')

    # Plot the generated path
    ax.plot([p.x for p in pnts], [p.y for p in pnts], [p.z for p in pnts], 'b')

    # Plot original waypoints
    ax.plot(waypoint_set.x, waypoint_set.y, waypoint_set.z, 'r.')

    # Plot the raw path along the waypoints
    ax.plot(waypoint_set.x, waypoint_set.y, waypoint_set.z, 'g--')

    for i in range(1, len(pnts), 100):
        p0 = pnts[i - 1]
        p1 = p0.pos + np.dot(p0.rot_matrix, [2, 0, 0])
        ax.plot([p0.pos[0], p1[0]], [p0.pos[1], p1[1]], [p0.pos[2], p1[2]], 'c', linewidth=2)
    ax.grid(True)
    ax.set_title(interp_method)

    # Position and orientation plot
    fig = plt.figure()
    ax = fig.add_subplot(211)
    for i in range(3):
        ax.plot([p.t for p in pnts], [p.pos[i] for p in pnts], label='%d' % i)
    ax.legend()
    ax.grid(True)
    ax.set_title('Position - ' + interp_method)
    ax.set_xlim(pnts[0].t, pnts[-1].t)

    ax = fig.add_subplot(212)
    for i in range(3):
        ax.plot([p.t for p in pnts], [p.rot[i] * 180 / np.pi for p in pnts], label='%d' % i)
    ax.legend()
    ax.grid(True)
    ax.set_title('Rotation - ' + interp_method)
    ax.set_xlim(pnts[0].t, pnts[-1].t)

    fig = plt.figure()
    ax = fig.add_subplot(211)
    for i in range(3):
        ax.plot([p.t for p in pnts], [p.vel[i] for p in pnts], label='%d' % i)
    ax.legend()
    ax.grid(True)
    ax.set_title('Linear velocity - ' + interp_method)
    ax.set_xlim(pnts[0].t, pnts[-1].t)

    ax = fig.add_subplot(212)
    for i in range(3):
        ax.plot([p.t for p in pnts], [p.vel[i+3] for p in pnts], label='%d' % i)
    ax.legend()
    ax.grid(True)
    ax.set_title('Angular velocity - ' + interp_method)
    ax.set_xlim(pnts[0].t, pnts[-1].t)

    fig = plt.figure()
    ax = fig.add_subplot(211)
    for i in range(3):
        ax.plot([p.t for p in pnts], [p.acc[i] for p in pnts], label='%d' % i)
    ax.legend()
    ax.grid(True)
    ax.set_title('Linear accelerations - ' + interp_method)
    ax.set_xlim(pnts[0].t, pnts[-1].t)

    ax = fig.add_subplot(212)
    for i in range(3):
        ax.plot([p.t for p in pnts], [p.acc[i+3] for p in pnts], label='%d' % i)
    ax.legend()
    ax.grid(True)
    ax.set_title('Angular accelerations - ' + interp_method)
    ax.set_xlim(pnts[0].t, pnts[-1].t)

#==============================================================================
def main():
    # For a helical trajectory
    wp_set = WaypointSet()
    # Add some waypoints at the beginning
    wp_set.add_waypoint(Waypoint(-10, -12, -36, 0.5),
                        add_to_beginning=True)
    wp_set.add_waypoint(Waypoint(-13, -15, -44, 0.5),
                        add_to_beginning=True)
    wp_set.add_waypoint(Waypoint(-20, -24, -48, 0.5),
                        add_to_beginning=True)
    wp_set.add_waypoint(Waypoint(-10, 10, -5, 0.5))
    wp_set.add_waypoint(Waypoint(-20, 20, -5, 0.5))
    wp_set.add_waypoint(Waypoint(-30, 60, -50, 0.5))
    wp_set.add_waypoint(Waypoint(-40, 70, -55, 0.5))
    wp_set.add_waypoint(Waypoint(-40, 80, -30, 0.5))

    run_generator(wp_set, 'cubic')
    run_generator(wp_set, 'lipb')

    plt.show()

#==============================================================================
if __name__ == '__main__':
    main()
