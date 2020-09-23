# Copyright (c) 2020 The Plankton Authors.
# All rights reserved.
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

LOGGER = None

def get_logger():
    global LOGGER
    if LOGGER is None:
        import logging
        import sys
        
        LOGGER = logging.getLogger('uuv_control_interfaces')
        out_hdlr = logging.StreamHandler(sys.stdout)
        out_hdlr.setFormatter(logging.Formatter(' %(asctime)s | %(levelname)s | %(module)s | %(message)s'))
        out_hdlr.setLevel(logging.INFO)
        LOGGER.addHandler(out_hdlr)
        LOGGER.setLevel(logging.INFO)

    return LOGGER
