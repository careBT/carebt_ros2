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

import threading
import json
import rclpy

from carebt_msgs.srv import KbQuery
from carebt_msgs.action import KbEvalState
from carebt_kb.owlready2_kb import OwlReady2Kb
from carebt_kb.plugin_base import import_class

from rclpy.action import ActionServer, CancelResponse
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.action.server import ServerGoalHandle

from typing import List, Dict, Any



# parameter constants
KB_FILE_PARAM = 'kb_file'
KB_PERSIST_PARAM = 'kb_persist'
KB_PLUGIN_LIST_PARAM = 'plugins'


class KbServer(Node):

    def __init__(self, node_name: str):
        super().__init__(node_name)

        self.__pending_goals = []
        self.__pending_lock = threading.Lock()

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
        self.__kb = OwlReady2Kb(kb_file, kb_persist)
        self.get_logger().info(f'kb created.')

        # create reentrant callback group for all service callbacks
        self.reentrant_cb_group = ReentrantCallbackGroup()

        # create crud service
        self.create_service(KbQuery, 'carebt_kb/query', self.__crud_query_callback, callback_group=self.reentrant_cb_group)

        # create wait_state action server
        ActionServer(
            self,
            KbEvalState,
            'carebt_kb/wait_eval_state',
            callback_group=ReentrantCallbackGroup(),
            cancel_callback=self.__wait_eval_state_cancel_callback,
            handle_accepted_callback=self.__handle_accepted_callback)

        # create plugins
        self.__plugins = []
        for plugin in filter(lambda name: len(name) >= 3, self.get_parameter(KB_PLUGIN_LIST_PARAM).get_parameter_value().string_array_value):
            self.declare_parameter(f'{plugin}.class', '')
            plugin_type = self.get_parameter(f'{plugin}.class').get_parameter_value().string_value
            self.get_logger().info(f'init plugin: {plugin} - {plugin_type}')

            # import and instantiate plugin
            plugin_class = import_class(plugin_type)
            self.__plugins.append(plugin_class(self, plugin))

        # exclusive group ensures a single evaluator; repeated triggers coalesce
        self.__eval_trigger = self.create_guard_condition(
            self.__evaluate_pending_goals,
            callback_group=MutuallyExclusiveCallbackGroup())

    def __kb_updated(self):
        self.__eval_trigger.trigger()
        for plugin in self.__plugins:
            plugin.on_update_callback()

    ## wait_eval_state action-server callbacks

    def __handle_accepted_callback(self, goal_handle: ServerGoalHandle):
        """Store goal for pooled evaluation."""
        self.get_logger().info(f'handle_accepted_callback -- Received goal: {goal_handle.request.filter}, {goal_handle.request.eval}')
        goal_handle.executing()
        with self.__pending_lock:
            self.__pending_goals.append(goal_handle)
        self.__eval_trigger.trigger()

    def __evaluate_pending_goals(self):
        with self.__pending_lock:
            goals = list(self.__pending_goals)

        if not goals:
            return

        completed = []
        for goal_handle in goals:
            if not goal_handle.is_active:
                completed.append(goal_handle)
                continue

            if goal_handle.is_cancel_requested:
                try:
                    goal_handle.canceled(KbEvalState.Result())
                    self.get_logger().info('evaluate -- Goal canceled')
                except Exception:
                    pass
                completed.append(goal_handle)
                continue

            goal = goal_handle.request
            filter_dict = json.loads(goal.filter)
            # Hold KB lock across read+eval to prevent false positives from stale data
            with self.__kb.lock:
                result = self.read(filter_dict)
                try:
                    eval_result = eval(goal.eval)
                except Exception as e:
                    eval_result = None
                    msg = f'eval: {goal.eval} -- EXCEPTION: {e}'
                    self.get_logger().warn(msg)
                    self.get_logger().warn(f'result= {result}')
                    feedback_msg = KbEvalState.Feedback()
                    feedback_msg.message = msg
                    try:
                        goal_handle.publish_feedback(feedback_msg)
                    except Exception:
                        completed.append(goal_handle)
            if eval_result:
                try:
                    goal_handle.succeed(KbEvalState.Result())
                    self.get_logger().info(
                        f'evaluate -- Goal succeeded: {goal.filter}, {goal.eval}')
                except Exception:
                    pass
                completed.append(goal_handle)

        if completed:
            with self.__pending_lock:
                for g in completed:
                    self.__pending_goals.remove(g)

    def __wait_eval_state_cancel_callback(self, goal_handle):
        self.get_logger().info(f'cancel_callback -- Received cancel request: {goal_handle}')
        self.__eval_trigger.trigger()
        return CancelResponse.ACCEPT

    ## CRUD query callback

    def __crud_query_callback(self, request: KbQuery.Request, response: KbQuery.Response):
        if request.operation == 'READ':
            self.get_logger().debug(
                f'Incoming request: {request.operation}, filter: {request.filter}')
        else:
            self.get_logger().info(
                f'Incoming request: {request.operation}, filter: {request.filter}')

        # create
        if(request.operation.upper() == 'CREATE'):
            frame = json.loads(request.data)
            result = []
            result.append(self.create(frame))
            response.response = json.dumps(result)
        # read
        elif(request.operation.upper() == 'READ'):
            filter = json.loads(request.filter)
            result = self.read(filter)
            response.response = json.dumps(result)
        # read_items
        elif(request.operation.upper() == 'READ_ITEMS'):
            items = json.loads(request.filter)['items']
            result = self.read_items(items)
            response.response = json.dumps(result)
        # update
        elif(request.operation.upper() == 'UPDATE'):
            filter = json.loads(request.filter)
            update = json.loads(request.data)
            result = self.update(filter, update)
            response.response = json.dumps(result)
        # update_items
        elif(request.operation.upper() == 'UPDATE_ITEMS'):
            items = json.loads(request.filter)['items']
            update = json.loads(request.data)
            result = self.update_items(items, update)
            response.response = json.dumps(result)
        # delete
        elif(request.operation.upper() == 'DELETE'):
            filter = json.loads(request.filter)
            self.delete(filter)
            result = []
            response.response = json.dumps(result)
        # delete_items
        elif(request.operation.upper() == 'DELETE_ITEMS'):
            items = json.loads(request.filter)['items']
            result = self.delete_items(items)
            result = []
            response.response = json.dumps(result)
        else:
            print(f'unsupported operation ({request.operation}), '
                  + 'use: CREATE/READ/READ_ITEMS/UPDATE/UPDATE_ITEMS/DELETE/DELETE_ITEMS')

        return response

    ## the Kb CRUD operations

    def create(self, frame) -> str:
        item = self.__kb.create(frame)
        self.__kb_updated()
        return item

    def read(self, filter):
        return self.__kb.read(filter)

    def read_items(self, items):
        return self.__kb.read_items(items)

    def update(self, filter, update):
        self.__kb.update(filter, update)
        self.__kb_updated()
        return self.__kb.read(filter)

    def update_items(self, items, update):
        self.__kb.update_items(items, update)
        self.__kb_updated()
        return self.__kb.read_items(items)

    def delete(self, filter) -> None:
        self.__kb.delete(filter)
        self.__kb_updated()

    def delete_items(self, items) -> None:
        self.__kb.delete_items(items)
        self.__kb_updated()

    ## 

    def get_classes(self):
        return self.__kb.get_classes()

    def has_subclasses(self, class_str: str):
        return self.__kb.has_subclasses(class_str)

    def get_subclasses_of(self, class_str: str):
        return self.__kb.get_subclasses_of(class_str)

    def get_individuals_of(self, class_str: str):
        return self.__kb.get_individuals_of(class_str)

    def is_individual_of(self, individual_str: str, class_str: str) -> bool:
        return self.__kb(individual_str, class_str)

    def get_properties_of_class(self, class_str: str) -> List[Dict[str, str | bool]]:
        return self.__kb.get_properties_of_class(class_str)


def main(args=None):
    rclpy.init(args=args)
    node = KbServer('carebt_kb')

    executor = MultiThreadedExecutor()
    rclpy.spin(node, executor=executor)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
