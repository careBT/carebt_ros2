# Copyright 2022 Andreas Steck (steck.andi@gmail.com)
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

import math

from carebt_nav2_pyutil.geometry_utils import euclidean_distance
from geometry_msgs.msg import Pose


class Test_euclidean_distance():

    def test_euclidean_distance(self):
        p1 = Pose()
        p1.position.x = 1.0
        p1.position.y = 1.0
        p2 = Pose()
        p2.position.x = 5.0
        p2.position.y = 5.0
        assert math.isclose(euclidean_distance(p1, p2), 5.656854249492381)
