# Copyright 2021 Andreas Steck (steck.andi@gmail.com)
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

from carebt_msgs.srv import RequestResponse
from carebt_simple_kb.carebt_simple_kb import KbServer
from geometry_msgs.msg import PoseStamped
import json
import math
import pytest
import rclpy
from rclpy_message_converter import message_converter


class TestKbServer():

    @pytest.fixture(scope="class", autouse=True)
    def execute_before_any_test(self):
        rclpy.init() 

    def test_read_bob(self, execute_before_any_test):
        kbserver = KbServer()
        req = RequestResponse.Request()
        res = RequestResponse.Response()

        req.operation = 'READ'
        filter = {'is-a': 'Person', 'name': 'Bob'}
        req.filter = json.dumps(filter)
        result = kbserver.query_callback(req, res)
        result = json.loads(result.response)
        
        assert len(result) == 1
        assert len(result[0]) == 3
        assert result[0]['is-a'] == 'Person'
        assert result[0]['name'] == 'Bob'
        assert result[0]['age'] == 39

    def test_read_xxx(self, execute_before_any_test):
        kbserver = KbServer()
        req = RequestResponse.Request()
        res = RequestResponse.Response()

        req.operation = 'READ'
        filter = {'is-a': 'Person', 'name': 'XXX'}
        req.filter = json.dumps(filter)
        result = kbserver.query_callback(req, res)
        result = json.loads(result.response)
        
        assert len(result) == 0 

    def test_update_age_of_bob(self, execute_before_any_test):
        kbserver = KbServer()
        req = RequestResponse.Request()
        res = RequestResponse.Response()
    
        req.operation = 'UPDATE'
        filter = {'is-a': 'Person', 'name': 'Bob'}
        data = {'age': 55}
        req.operation = 'UPDATE'
        req.filter = json.dumps(filter)
        req.data = json.dumps(data)

        result = kbserver.query_callback(req, res)
        result = json.loads(result.response)

        assert len(result) == 1
        assert len(result[0]) == 3
        assert result[0]['is-a'] == 'Person'
        assert result[0]['name'] == 'Bob'
        assert result[0]['age'] == 55

    def test_update_add_pose_to_bob(self, execute_before_any_test):
        kbserver = KbServer()
        req = RequestResponse.Request()
        res = RequestResponse.Response()
    
        req.operation = 'UPDATE'
        filter = {'is-a': 'Person', 'name': 'Bob'}
        p = PoseStamped()
        p.pose.position.x = 1.0
        p.pose.position.y = 2.0
        p_dict = message_converter.convert_ros_message_to_dictionary(p)
        data = {'pose': p_dict}
        req.operation = 'UPDATE'
        req.filter = json.dumps(filter)
        req.data = json.dumps(data)

        result = kbserver.query_callback(req, res)
        result = json.loads(result.response)
        
        p : PoseStamped = message_converter.convert_dictionary_to_ros_message(
            'geometry_msgs/msg/PoseStamped', result[0]['pose'])

        assert len(result) == 1
        assert len(result[0]) == 4
        assert math.isclose(result[0]['pose']['pose']['position']['x'], 1.0)
        assert math.isclose(result[0]['pose']['pose']['position']['y'], 2.0)
        assert math.isclose(p.pose.position.x, 1.0)
        assert math.isclose(p.pose.position.y, 2.0)
