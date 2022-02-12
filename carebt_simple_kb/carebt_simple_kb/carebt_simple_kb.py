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
from carebt_msgs.action import KbEvalState
from carebt_simple_kb.simple_kb import SimpleKb
from carebt_simple_kb.plugin_base import import_class
import json
import rclpy
from rclpy.action import ActionServer, CancelResponse
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.action.server import ServerGoalHandle
import time


# parameter constants
KB_FILE_PARAM = 'kb_file'
KB_PERSIST_PARAM = 'kb_persist'
KB_PLUGIN_LIST_PARAM = 'plugins'


class KbServer(Node):

    def __init__(self, node_name: str):
        super().__init__(node_name)

        # declare parameters
        self.declare_parameter(KB_FILE_PARAM, 'memory.json')
        self.declare_parameter(KB_PERSIST_PARAM, False)
        self.declare_parameter(KB_PLUGIN_LIST_PARAM, [''])  # TODO: empty string list

        # read parameters
        kb_file = self.get_parameter(KB_FILE_PARAM).get_parameter_value().string_value
        kb_persist = self.get_parameter(KB_PERSIST_PARAM).get_parameter_value().bool_value

        # create SimpleKb
        self.get_logger().info(f'create kb from file: {kb_file}, persist = {kb_persist}')
        self.__simple_kb = SimpleKb(kb_file, kb_persist)
        self.get_logger().info(f'kb created with {self.__simple_kb.size()} bytes; '
                               + f'{self.__simple_kb.count()} entries')

        # create crud service
        self.create_service(KbCrud, 'crud', self.query_callback)

        # create wait_state action server
        ActionServer(
            self,
            KbEvalState,
            'wait_state',
            callback_group=ReentrantCallbackGroup(),
            execute_callback=self.__wait_state_execute_callback,
            #goal_callback=self.__wait_state_goal_callback,
            cancel_callback=self.__wait_state_cancel_callback)

        # create plugins
        self.__plugins = []
        for plugin in filter(lambda name: len(name) >= 3, self.get_parameter(KB_PLUGIN_LIST_PARAM).get_parameter_value().string_array_value):
            self.declare_parameter(f'{plugin}.class', '')
            plugin_type = self.get_parameter(f'{plugin}.class').get_parameter_value().string_value
            self.get_logger().info(f'init plugin: {plugin} - {plugin_type}')

            # import and instantiate plugin
            plugin_class = import_class(plugin_type)
            self.__plugins.append(plugin_class(self, plugin))

    def create(self, item) -> None:
        self.__simple_kb.create(item)

    def read(self, filter):
        return self.__simple_kb.read(filter)

    def update(self, filter, update) -> None:
        self.__simple_kb.update(filter, update)
        return self.__simple_kb.read(filter)

    def delete(self, filter):
        self.__simple_kb.delete(filter)

    # def __wait_state_goal_callback(self, goal_request):
    #     self.get_logger().info(f'goal_callback -- Received goal request: {goal_request}')
    #     self.__wait_state_goal_requests.append(goal_request)
    #     return GoalResponse.ACCEPT

    def __wait_state_execute_callback(self, goal_handle: ServerGoalHandle):
        while True:
            goal: KbEvalState.Goal = goal_handle.request
            if not goal_handle.is_active:
                self.get_logger().info("execute_callback -- Goal aborted")
                return KbEvalState.Result()

            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info("execute_callback -- Goal canceled")
                return KbEvalState.Result()

            filter = json.loads(goal.filter)
            result = self.read(filter)
            print(f'result= {result}')
            try:
                if(eval(goal.eval)):
                    print(f'eval {goal.eval} is true')
                    break
            except Exception as e:
                print(f'eval {goal.eval} exception: {e}')

            time.sleep(1)

        goal_handle.succeed()
        result = KbEvalState.Result()
        result.response = 'eval is True'
        return result

    def __wait_state_cancel_callback(self, goal_handle):
        self.get_logger().info(f'cancel_callback -- Received cancel request: {goal_handle}')
        return CancelResponse.ACCEPT

    def query_callback(self, request: KbCrud.Request, response: KbCrud.Response):
        self.get_logger().info(
            f'Incoming request: {request.operation} {request.filter} {request.data}')

        # create
        if(request.operation.upper() == 'CREATE'):
            item = json.loads(request.data)
            self.create(item)
            result = []
            response.response = json.dumps(result)
        # read
        elif(request.operation.upper() == 'READ'):
            filter = json.loads(request.filter)
            result = self.read(filter)
            response.response = json.dumps(result)
        # update
        elif(request.operation.upper() == 'UPDATE'):
            filter = json.loads(request.filter)
            update = json.loads(request.data)
            result = self.update(filter, update)
            response.response = json.dumps(result)
        # delete
        elif(request.operation.upper() == 'DELETE'):
            filter = json.loads(request.filter)
            self.delete(filter)
            result = []
            response.response = json.dumps(result)
        else:
            print(f'unsupported operation ({request.operation}), use: CREATE/READ/UPDATE/DELETE')

        return response


def main(args=None):
    rclpy.init(args=args)
    node = KbServer('carebt_simple_kb')

    # Use a MultiThreadedExecutor to enable processing goals concurrently
    executor = MultiThreadedExecutor()
    rclpy.spin(node, executor=executor)

    rclpy.shutdown()


if __name__ == '__main__':
    main()