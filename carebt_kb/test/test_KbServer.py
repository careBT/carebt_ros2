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

from carebt_msgs.srv import KbCrud
from carebt_kb.carebt_kb import KbServer
from geometry_msgs.msg import PoseStamped
import json
import math
import pytest
import rclpy
from rclpy_message_converter import message_converter


class TestKbServer():

    @pytest.fixture(scope="class", autouse=True)
    def execute_before_any_test(self):
        rclpy.init(args=['carebt_kb', '--ros-args',
                         '-p', 'kb_file:=src/carebt_ros2/carebt_kb/test/data/person.owl',
                         '-p', 'kb_persist:=False'])

    def test_read_bob(self, execute_before_any_test):
        kbserver = KbServer('carebt_kb')
        req = KbCrud.Request()
        res = KbCrud.Response()

        req.operation = 'READ'
        filter = {'type': 'person.Person', 'first_name': 'Bob'}
        req.filter = json.dumps(filter)
        result = kbserver._KbServer__crud_query_callback(req, res)
        result = json.loads(result.response)

        assert len(result) == 1
        assert len(result[0]) == 6
        assert result[0]['is_a'] == ['person.Person']
        assert result[0]['first_name'] == 'Bob'
        assert result[0]['age'] == 21
        assert math.isclose(result[0]['size'], 1.8)
        assert math.isclose(result[0]['weight'], 95.0)

    def test_read_xxx(self, execute_before_any_test):
        kbserver = KbServer('carebt_kb')
        req = KbCrud.Request()
        res = KbCrud.Response()

        req.operation = 'READ'
        filter = {'type': 'person.Person', 'first_name': 'XXX'}
        req.filter = json.dumps(filter)
        result = kbserver._KbServer__crud_query_callback(req, res)
        result = json.loads(result.response)

        assert len(result) == 0

    def test_update_age_of_bob(self, execute_before_any_test):
        kbserver = KbServer('carebt_kb')
        req = KbCrud.Request()
        res = KbCrud.Response()

        req.operation = 'UPDATE'
        filter = {'type': 'person.Person', 'first_name': 'Bob'}
        data = {'age': 55}
        req.operation = 'UPDATE'
        req.filter = json.dumps(filter)
        req.data = json.dumps(data)

        result = kbserver._KbServer__crud_query_callback(req, res)
        result = json.loads(result.response)

        assert len(result) == 1
        assert len(result[0]) == 6
        assert result[0]['is_a'] == ['person.Person']
        assert result[0]['first_name'] == 'Bob'
        assert result[0]['age'] == 55
        assert math.isclose(result[0]['size'], 1.8)
        assert math.isclose(result[0]['weight'], 95.0)
