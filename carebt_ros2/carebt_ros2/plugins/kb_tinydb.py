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

import sys

from carebt_msgs.srv import RequestResponse
from rclpy.node import Node
import tinydb


class Kb():

    def __init__(self,
                 node: Node,
                 kbfile: str):

        # create tinydb
        self.get_logger().info('create tinydb kb from file: {}'.format(kbfile))
        self._kb = tinydb.TinyDB(kbfile)

        node.create_service(RequestResponse, 'kb_tellask', self.query_callback)

    def query_callback(self, request: RequestResponse.Request, response: RequestResponse.Response):
        self.get_logger().info('request: {}'.format(request.request))
        try:
            exec('response.response = str(self._kb.{})'.format(request.request))
            self.get_logger().info('result: {}'.format(response.response))
        except (AttributeError, RuntimeError) as err:
            self.get_logger().error('an error occurred: {}\n{}'.format(str(sys.exc_info()[0]), err))
            response.response = str(sys.exc_info()[0])
        return response

    def get_kb(self) -> tinydb.TinyDB:
        return self._kb
