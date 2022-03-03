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
import json
from rclpy_message_converter import message_converter

def dict_from_kb_response(response: KbQuery.Response):
    return json.loads(response.response)

def kb_rosstr_from_ros_msg(ros_msg):
    msg_dict = message_converter.convert_ros_message_to_dictionary(ros_msg)
    return json.dumps(msg_dict).replace('"', '\\"')

def kb_rosstr_from_ros_msg_list(ros_msg_list: list):
    rosstr_list = []
    for ros_msg in ros_msg_list:
        msg_dict = message_converter.convert_ros_message_to_dictionary(ros_msg)
        rosstr_list.append(json.dumps(msg_dict))
    return rosstr_list

def create_create_request(frame: dict):
    req = KbQuery.Request()
    req.operation = 'CREATE'
    req.data = json.dumps(frame)
    return req

def create_search_request(filter: dict):
    req = KbQuery.Request()
    req.operation = 'SEARCH'
    req.filter = json.dumps(filter)
    return req

def create_read_request(items: list):
    req = KbQuery.Request()
    req.operation = 'READ'
    req.filter = json.dumps(f'"items": {items}')
    return req

def create_update_request(filter: dict, data: dict):
    req = KbQuery.Request()
    req.operation = 'UPDATE'
    req.filter = json.dumps(filter)
    req.data = json.dumps(data)
    return req

def create_delete_request(filter: dict):
    req = KbQuery.Request()
    req.operation = 'DELETE'
    req.filter = json.dumps(filter)
    return req
