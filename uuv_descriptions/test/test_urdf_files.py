#!/usr/bin/env python
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

import unittest
import subprocess
import os


def call_xacro(xml_file):
    assert os.path.isfile(xml_file), 'Invalid XML xacro file'
    return subprocess.check_output(['xacro', '--inorder', xml_file])


# =============================================================================
class TestRexROVURDFFiles(unittest.TestCase):
    def test_xacro(self):
        # Retrieve the root folder for the tests
        test_dir = os.path.abspath(os.path.dirname(__file__))
        robots_dir = os.path.join(test_dir, '..', 'robots')

        for item in os.listdir(robots_dir):
            if 'oberon' in item:
                continue
            if not os.path.isfile(os.path.join(robots_dir, item)):
                continue
            output = call_xacro(os.path.join(robots_dir, item))

            self.assertNotIn(
                'XML parsing error',
                output.decode('utf-8'), 
                'Parsing error found for file {}'.format('hey'))
            self.assertNotIn(
                'No such file or directory', 
                output.decode('utf-8'), 
                'Some file not found in {}'.format('hey'))




