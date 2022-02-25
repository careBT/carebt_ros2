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
from carebt_msgs.action import KbEvalState
from carebt_kb.owlready2_kb import OwlReady2Kb
from carebt_kb.plugin_base import import_class
import json
import rclpy
from rclpy.action import ActionServer, CancelResponse
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.action.server import ServerGoalHandle
import threading


# parameter constants
KB_FILE_PARAM = 'kb_file'
KB_PERSIST_PARAM = 'kb_persist'
KB_PLUGIN_LIST_PARAM = 'plugins'


class KbServer(Node):

    def __init__(self, node_name: str):
        super().__init__(node_name)

        self.__event = threading.Event()

        # declare parameters
        self.declare_parameter(KB_FILE_PARAM, 'src/carebt_ros2/carebt_kb/test/data/person.owl')
        self.declare_parameter(KB_PERSIST_PARAM, False)
        # TODO: empty string list, instead of list with one empty string
        self.declare_parameter(KB_PLUGIN_LIST_PARAM, [''])

        # read parameters
        kb_file = self.get_parameter(KB_FILE_PARAM).get_parameter_value().string_value
        kb_persist = self.get_parameter(KB_PERSIST_PARAM).get_parameter_value().bool_value

        # create SimpleKb
        self.get_logger().info(f'create kb from file: {kb_file}, persist = {kb_persist}')
        self.__simple_kb = OwlReady2Kb(kb_file, kb_persist)
        self.get_logger().info(f'kb created.')

        # create crud service
        self.create_service(KbQuery, 'crud', self.__crud_query_callback)

        # create wait_state action server
        ActionServer(
            self,
            KbEvalState,
            'wait_eval_state',
            callback_group=ReentrantCallbackGroup(),
            execute_callback=self.__wait_eval_state_execute_callback,
            cancel_callback=self.__wait_eval_state_cancel_callback)

        # create plugins
        self.__plugins = []
        for plugin in filter(lambda name: len(name) >= 3, self.get_parameter(KB_PLUGIN_LIST_PARAM).get_parameter_value().string_array_value):
            self.declare_parameter(f'{plugin}.class', '')
            plugin_type = self.get_parameter(f'{plugin}.class').get_parameter_value().string_value
            self.get_logger().info(f'init plugin: {plugin} - {plugin_type}')

            # import and instantiate plugin
            plugin_class = import_class(plugin_type)
            self.__plugins.append(plugin_class(self, plugin))

    def __kb_updated(self):
        self.__event.set()
        self.__event.clear()
        for plugin in self.__plugins:
            plugin.on_update_callback()

    ## wait_eval_state action-server callbacks

    def __wait_eval_state_execute_callback(self, goal_handle: ServerGoalHandle):
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
            try:
                if(eval(goal.eval)):
                    break
            except Exception as e:
                msg = f'eval: {goal.eval} -- EXCEPTION: {e}'
                self.get_logger().warn(msg)
                feedback_msg = KbEvalState.Feedback()
                feedback_msg.message = msg
                goal_handle.publish_feedback(feedback_msg)

            self.__event.wait()

        goal_handle.succeed()
        result = KbEvalState.Result()
        return result

    def __wait_eval_state_cancel_callback(self, goal_handle):
        self.get_logger().info(f'cancel_callback -- Received cancel request: {goal_handle}')
        return CancelResponse.ACCEPT

    ## CRUD query callback

    def __crud_query_callback(self, request: KbQuery.Request, response: KbQuery.Response):
        self.get_logger().info(
            f'Incoming request: {request.operation} {request.filter} {request.data}')

        # create
        if(request.operation.upper() == 'CREATE'):
            frame = json.loads(request.data)
            self.create(frame)
            result = []
            response.response = json.dumps(result)
        # search
        elif(request.operation.upper() == 'SEARCH'):
            filter = json.loads(request.filter)
            result = self.search(filter)
            response.response = json.dumps(result)
        # read
        elif(request.operation.upper() == 'READ'):
            filter = json.loads(request.filter)['items']
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

    ## the Kb CRUD operations

    def create(self, frame) -> None:
        self.__simple_kb.create(frame)
        self.__kb_updated()

    def search(self, filter):
        return self.__simple_kb.search(filter)

    def read(self, filter):
        return self.__simple_kb.read(filter)

    def update(self, filter, update) -> None:
        self.__simple_kb.update(filter, update)
        self.__kb_updated()
        return self.__simple_kb.search(filter)

    def delete(self, filter):
        self.__simple_kb.delete(filter)
        self.__kb_updated()


def main(args=None):
    rclpy.init(args=args)
    node = KbServer('carebt_kb')

    # Use a MultiThreadedExecutor to enable processing goals concurrently
    executor = MultiThreadedExecutor()
    rclpy.spin(node, executor=executor)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
