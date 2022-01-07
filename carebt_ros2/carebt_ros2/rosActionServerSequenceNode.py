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

import threading
from time import sleep
from typing import TYPE_CHECKING

from carebt.nodeStatus import NodeStatus
from carebt.sequenceNode import SequenceNode
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup

if TYPE_CHECKING:
    from carebt.behaviorTreeRunner import BehaviorTreeRunner  # pragma: no cover


class RosActionServerSequenceNode(SequenceNode):

    def __init__(self,
                 bt: 'BehaviorTreeRunner',
                 action_type,
                 action_name: str,
                 params: str = None):
        super().__init__(bt, params)
        self._succeed = False
        self._goal_handle = None
        self.__result = action_type.Result()
        self._goal_lock = threading.Lock()
        self._action_server = ActionServer(
            bt.node,
            action_type,
            action_name,
            callback_group=ReentrantCallbackGroup(),
            execute_callback=self.__execute_callback,
            goal_callback=self.__goal_callback,
            handle_accepted_callback=self.__handle_accepted_callback,
            cancel_callback=self.__cancel_callback)
        self.set_status(NodeStatus.SUSPENDED)

    def __execute_callback(self, goal_handle):
        self.execute_callback()
        # wait for _cond_succeed TODO: mutex oder so
        while(self._succeed is False):
            sleep(0.1)
        goal_handle.succeed()
        return self.__result

    def __goal_callback(self, goal_request):
        self.get_logger().debug('{} - __goal_callback -- Received goal request: {}'
                                .format(self.__class__.__name__, goal_request))
        return GoalResponse.ACCEPT

    # replace current goal with new goal
    def __handle_accepted_callback(self, goal_handle):
        with self._goal_lock:
            self.get_logger().debug('{} - __handle_accepted_callback'
                                    .format(self.__class__.__name__))
            if self._goal_handle is not None and self._goal_handle.is_active:
                self.get_logger().debug('{} - __handle_accepted_callback -- Destroy previous goal'
                                        .format(self.__class__.__name__))
                # destroy old _goal_handle
                self._goal_handle.destroy()
            self._goal_handle = goal_handle
            self._succeed = False
        goal_handle.execute()

    def __cancel_callback(self):
        self.get_logger().debug('{} - __cancel_callback'.format(self.__class__.__name__))
        return CancelResponse.ACCEPT

    # PUBLIC

    def execute_callback(self) -> None:
        pass

    def succeed(self) -> None:
        self._succeed = True
