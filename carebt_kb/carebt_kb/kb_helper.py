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
import json
from rclpy_message_converter import message_converter

def dict_from_response(response: KbCrud.Response):
    return json.loads(response.response)
    
def ros_msg_from_str(msg_type: str, ros_msg_str: str):
    return message_converter.convert_dictionary_to_ros_message(
            msg_type, eval(ros_msg_str))

def str_from_ros_msg(ros_msg):
    msg_dict = message_converter.convert_ros_message_to_dictionary(ros_msg)
    return json.dumps(msg_dict).replace('"', '\\"')

def create_create_request(frame: dict):
    req = KbCrud.Request
    req.operation = 'CREATE'
    req.filter = json.dumps(frame)
    return req

def create_read_request(filter: dict):
    req = KbCrud.Request
    req.operation = 'READ'
    req.filter = json.dumps(filter)
    return req

def create_update_request(filter: dict, data: dict):
    req = KbCrud.Request
    req.operation = 'UPDATE'
    req.filter = json.dumps(filter)
    req.data = json.dumps(data)
    return req

def create_delete_request(filter: dict):
    req = KbCrud.Request
    req.operation = 'DELETE'
    req.filter = json.dumps(filter)
    return req
