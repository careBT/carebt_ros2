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

from carebt_msgs.srv import KbQuery
from carebt_kb.carebt_kb import KbServer
from carebt_kb.kb_helper import create_search_request
from carebt_kb.kb_helper import create_update_request
from carebt_kb.kb_helper import json_dict_str_from_ros_msg
from carebt_kb.kb_helper import ros_msg_from_dict_str
from carebt_kb.kb_helper import dict_from_kb_response
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
                         '-p', 'kb_file:=src/carebt_ros2/carebt_kb/test/data/demo1.owl',
                         '-p', 'kb_persist:=False'])

    def test_read_bob(self, execute_before_any_test):
        kbserver = KbServer('carebt_kb')

        filter = {'type': 'demo1.Person', 'first_name': 'Bob'}
        req = create_search_request(filter)
        res = kbserver._KbServer__crud_query_callback(req, KbQuery.Response())
        result = dict_from_kb_response(res)

        assert len(result) == 1
        assert len(result[0]) == 6
        assert result[0]['is_a'] == ['demo1.Person']
        assert result[0]['first_name'] == 'Bob'
        assert result[0]['age'] == 21
        assert math.isclose(result[0]['size'], 1.8)
        assert math.isclose(result[0]['weight'], 95.0)

    def test_read_xxx(self, execute_before_any_test):
        kbserver = KbServer('carebt_kb')

        filter = {'type': 'demo1.Person', 'first_name': 'XXX'}
        req = create_search_request(filter)
        res = kbserver._KbServer__crud_query_callback(req, KbQuery.Response())
        result = dict_from_kb_response(res)

        assert len(result) == 0

    def test_update_age_of_bob(self, execute_before_any_test):
        kbserver = KbServer('carebt_kb')

        filter = {'type': 'demo1.Person', 'first_name': 'Bob'}
        data = {'age': 55}
        req = create_update_request(filter, data)
        res = kbserver._KbServer__crud_query_callback(req, KbQuery.Response())
        result = dict_from_kb_response(res)

        assert len(result) == 1
        assert len(result[0]) == 6
        assert result[0]['is_a'] == ['demo1.Person']
        assert result[0]['first_name'] == 'Bob'
        assert result[0]['age'] == 55
        assert math.isclose(result[0]['size'], 1.8)
        assert math.isclose(result[0]['weight'], 95.0)

    def test_update_add_pose_to_robot_1(self, execute_before_any_test):
        kbserver = KbServer('carebt_kb')

        filter = {'type': 'demo1.Robot', 'robot_id': 1}
        p = PoseStamped()
        p.pose.position.x = 1.0
        p.pose.position.y = 2.0
        data = {'pose_rosstr': json_dict_str_from_ros_msg(p), 'status': 'Happy'}
        print(f'### data:  {data}')
        req = create_update_request(filter, data)
        res = kbserver._KbServer__crud_query_callback(req, KbQuery.Response())
        result = dict_from_kb_response(res)

        print('result', result)
        print('result[0]["pose_rosstr"]', result[0]['pose_rosstr'])
        p: PoseStamped = ros_msg_from_dict_str('geometry_msgs/msg/PoseStamped', result[0]['pose_rosstr'])
        
        assert len(result) == 1
        assert len(result[0]) == 5
        assert result[0]['robot_id'] == 1
        assert result[0]['is_a'] == ['demo1.Robot']
        assert result[0]['status'] == 'Happy'
        assert isinstance(result[0]['pose_rosstr'], str)
        assert math.isclose(p.pose.position.x, 1.0)
        assert math.isclose(p.pose.position.y, 2.0)
