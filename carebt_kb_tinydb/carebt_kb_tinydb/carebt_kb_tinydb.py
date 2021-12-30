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

import sys

from carebt_msgs.srv import TellAsk
from rcl_interfaces.msg import SetParametersResult
import rclpy
from rclpy.node import Node
from tinydb import *


class KbQueryService(Node):

    def __init__(self):
        super().__init__('carebt_kb_tinydb')

        # create tinydb
        self.declare_parameter('kbfile_path', 'db.json')
        self._kbfile_path = self.get_parameter('kbfile_path').get_parameter_value().string_value
        self.get_logger().info('create tinydb kb from file: {}'.format(self._kbfile_path))
        self._db = TinyDB(self._kbfile_path)

        self.create_service(TellAsk, 'tellask', self.query_callback)
        self.add_on_set_parameters_callback(self.parameters_callback)

    def parameters_callback(self, params):
        for param in params:
            self.get_logger().info('parameters_callback for param: {} = {}'
                                   .format(param.name, param.value))
            if(param.name == 'kbfile_path'
               and param.value != self._kbfile_path):
                # close current tinydb
                self.get_logger().info('close tinydb kb: {}'.format(self._kbfile_path))
                self._db.close()

                # create new tinydb
                self._kbfile_path = param.value
                self.get_logger().info('create tinydb kb from file: {}'.format(self._kbfile_path))
                self._db = TinyDB(self._kbfile_path)
        return SetParametersResult(successful=True)

    def query_callback(self, request, response):
        self.get_logger().info('request: {}'.format(request.request))
        try:
            exec('response.result = str(self._db.{})'.format(request.request))
            response.success = True
            self.get_logger().info('result: {}'.format(response.result))
        except (AttributeError, RuntimeError) as err:
            self.get_logger().error('an error occured: {}\n{}'.format(str(sys.exc_info()[0]), err))
            response.success = False
            response.result = str(sys.exc_info()[0])
        return response


def main(args=None):
    rclpy.init(args=args)

    kb_query_service = KbQueryService()

    rclpy.spin(kb_query_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
