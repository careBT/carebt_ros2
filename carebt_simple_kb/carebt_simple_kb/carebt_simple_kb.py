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
from carebt_simple_kb.simple_kb import SimpleKb
import json
import rclpy
from rclpy.node import Node


KB_FILE_PARAM = 'kb_file'
KB_PERSIST_PARAM = 'kb_persist'


class KbServer(Node):

    def __init__(self, node_name: str):
        super().__init__(node_name)

        # declare parameters
        self.declare_parameter(KB_FILE_PARAM, 'memory.json')
        self.declare_parameter(KB_PERSIST_PARAM, False)

        # read parameters
        kb_file = self.get_parameter(KB_FILE_PARAM).get_parameter_value().string_value
        kb_persist = self.get_parameter(KB_PERSIST_PARAM).get_parameter_value().bool_value

        # create SimpleKb
        self.get_logger().info(f'create kb from file: {kb_file}, persist = {kb_persist}')
        self.__simple_kb = SimpleKb(kb_file, kb_persist)
        self.get_logger().info(f'kb created with {self.__simple_kb.size_in_kb()} bytes; '
            + f'{self.__simple_kb.count()} entries')

        # create ports
        self.srv = self.create_service(KbCrud, 'crud', self.query_callback)

    def query_callback(self, request: KbCrud.Request, response: KbCrud.Response):
        self.get_logger().info(
            f'Incoming request: {request.operation} {request.filter} {request.data}')

        # create
        if(request.operation.upper() == 'CREATE'):
            data = json.loads(request.data)
            self.__simple_kb.create(data)
            result = []
            response.response = json.dumps(result)
        # read
        elif(request.operation.upper() == 'READ'):
            filter = json.loads(request.filter)
            result = self.__simple_kb.read(filter)
            response.response = json.dumps(result)
        # update
        elif(request.operation.upper() == 'UPDATE'):
            filter = json.loads(request.filter)
            update = json.loads(request.data)
            self.__simple_kb.update(filter, update)
            result = self.__simple_kb.read(filter)
            response.response = json.dumps(result)
        # delete
        elif(request.operation.upper() == 'DELETE'):
            filter = json.loads(request.filter)
            self.__simple_kb.delete(filter)
            result = []
            response.response = json.dumps(result)
        else:
            print(f'unsupported operation ({request.operation}), use: CREATE/READ/UPDATE/DELETE')

        return response


def main(args=None):
    rclpy.init(args=args)
    node = KbServer('carebt_simple_kb')
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
