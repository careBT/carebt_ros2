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
from carebt_kb.kb_helper import create_create_request
from carebt_kb.kb_helper import create_read_request, create_read_items_request
from carebt_kb.kb_helper import create_update_request, create_update_items_request
from carebt_kb.kb_helper import create_delete_request, create_delete_items_request
from carebt_kb.kb_helper import kb_rosstr_from_ros_msg
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

    def test_create_grace(self, execute_before_any_test):
        kbserver = KbServer('carebt_kb')

        # check Grace
        filter = {'type': 'demo1.Person', 'first_name': 'Grace'}
        req = create_read_request(filter)
        res = kbserver._KbServer__crud_query_callback(req, KbQuery.Response())
        result = dict_from_kb_response(res)
        assert len(result) == 0

        # create
        frame = {'type': 'demo1.Person', 'first_name': 'Grace', 'age': 25, 'size': 1.66, 'weight': 52.5}
        req = create_create_request(frame)
        kbserver._KbServer__crud_query_callback(req, KbQuery.Response())

        # read Grace
        filter = {'type': 'demo1.Person', 'first_name': 'Grace'}
        req = create_read_request(filter)
        res = kbserver._KbServer__crud_query_callback(req, KbQuery.Response())
        result = dict_from_kb_response(res)
        assert len(result) == 1
        assert len(result[0]) == 6
        assert result[0]['is_a'] == ['demo1.Person']
        assert result[0]['first_name'] == 'Grace'

    def test_read_bob(self, execute_before_any_test):
        kbserver = KbServer('carebt_kb')

        filter = {'type': 'demo1.Person', 'first_name': 'Bob'}
        req = create_read_request(filter)
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
        req = create_read_request(filter)
        res = kbserver._KbServer__crud_query_callback(req, KbQuery.Response())
        result = dict_from_kb_response(res)

        assert len(result) == 0

    def test_read_item_bob(self, execute_before_any_test):
        kbserver = KbServer('carebt_kb')

        items = ['demo1.person2']
        req = create_read_items_request(items)
        res = kbserver._KbServer__crud_query_callback(req, KbQuery.Response())
        result = dict_from_kb_response(res)

        assert len(result) == 1
        assert len(result[0]) == 6
        assert result[0]['is_a'] == ['demo1.Person']
        assert result[0]['first_name'] == 'Bob'
        assert result[0]['age'] == 21
        assert math.isclose(result[0]['size'], 1.8)
        assert math.isclose(result[0]['weight'], 95.0)

    def test_read_items_alice_bob(self, execute_before_any_test):
        kbserver = KbServer('carebt_kb')

        items = ['demo1.person1', 'demo1.person2']
        req = create_read_items_request(items)
        res = kbserver._KbServer__crud_query_callback(req, KbQuery.Response())
        result = dict_from_kb_response(res)

        assert len(result) == 2
        assert result[0]['is_a'] == ['demo1.Person']
        assert result[0]['first_name'] == 'Alice'
        assert result[1]['is_a'] == ['demo1.Person']
        assert result[1]['first_name'] == 'Bob'
        

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

    def test_update_items_age_of_bob(self, execute_before_any_test):
        kbserver = KbServer('carebt_kb')

        items = ['demo1.person2']
        data = {'age': 55}
        req = create_update_items_request(items, data)
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
        data = {'pose_rosstr': kb_rosstr_from_ros_msg(p), 'status': 'Happy'}
        print(f'### data:  {data}')
        req = create_update_request(filter, data)
        res = kbserver._KbServer__crud_query_callback(req, KbQuery.Response())
        result = dict_from_kb_response(res)

        print('result', result)
        print('result[0]["pose_rosstr"]', result[0]['pose_rosstr'])
        p: PoseStamped = message_converter.convert_dictionary_to_ros_message('geometry_msgs/msg/PoseStamped', result[0]['pose_rosstr'])
        
        assert len(result) == 1
        assert len(result[0]) == 5
        assert result[0]['robot_id'] == 1
        assert result[0]['is_a'] == ['demo1.Robot']
        assert result[0]['status'] == 'Happy'
        assert isinstance(result[0]['pose_rosstr'], dict)
        assert math.isclose(p.pose.position.x, 1.0)
        assert math.isclose(p.pose.position.y, 2.0)

    def test_delete_bob(self, execute_before_any_test):
        kbserver = KbServer('carebt_kb')

        items = ['demo1.person2']
        req = create_read_items_request(items)
        res = kbserver._KbServer__crud_query_callback(req, KbQuery.Response())
        result = dict_from_kb_response(res)
        assert len(result) == 1

        filter = {'type': 'demo1.Person', 'first_name': 'Bob'}
        req = create_delete_request(filter)
        kbserver._KbServer__crud_query_callback(req, KbQuery.Response())
        
        items = ['demo1.person2']
        req = create_read_request(items)
        res = kbserver._KbServer__crud_query_callback(req, KbQuery.Response())
        result = dict_from_kb_response(res)
        assert len(result) == 0

    def test_delete_items_alice_bob(self, execute_before_any_test):
        kbserver = KbServer('carebt_kb')

        items = ['demo1.person1', 'demo1.person2']
        req = create_read_items_request(items)
        res = kbserver._KbServer__crud_query_callback(req, KbQuery.Response())
        result = dict_from_kb_response(res)
        assert len(result) == 2

        items = ['demo1.person1', 'demo1.person2']
        req = create_delete_items_request(items)
        kbserver._KbServer__crud_query_callback(req, KbQuery.Response())
        
        items = ['demo1.person1', 'demo1.person2']
        req = create_read_request(items)
        res = kbserver._KbServer__crud_query_callback(req, KbQuery.Response())
        result = dict_from_kb_response(res)
        assert len(result) == 0
