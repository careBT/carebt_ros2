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

from datetime import datetime
from typing import TYPE_CHECKING

from carebt.actionNode import ActionNode
from carebt.nodeStatus import NodeStatus
from rclpy.action import ActionClient

if TYPE_CHECKING:
    from carebt.behaviorTreeRunner import BehaviorTreeRunner  # pragma: no cover


class RosActionClientActionNode(ActionNode):

    def __init__(self,
                 bt_runner: 'BehaviorTreeRunner',
                 action_type,
                 action_name: str,
                 params: str = None):
        super().__init__(bt_runner, params)
        self.set_status(NodeStatus.IDLE)
        self._goal_msg = action_type.Goal()
        self._action_client = ActionClient(bt_runner.node, action_type, action_name)
        self.get_logger().debug('{} - action_client.wait_for_server...'
                                .format(self.__class__.__name__))
        self._action_client.wait_for_server()  # TODO: Timeout
        self.get_logger().debug('{} - action_client.wait_for_server -> OK'
                                .format(self.__class__.__name__))

    # PRIVATE

    def __goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().debug('{} - Goal rejected'.format(self.__class__.__name__))
            return

        self.get_logger().debug('{} - Goal accepted'.format(self.__class__.__name__))

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.result_callback)

    # PROTECTED

    def _internal_on_tick(self) -> None:
        current_ts = datetime.now()
        if(self._throttle_ms is None or
                int((current_ts - self._last_ts).total_seconds() * 1000) >= self._throttle_ms):
            if(self.get_status() == NodeStatus.IDLE or
                    self.get_status() == NodeStatus.RUNNING):
                self.bt_runner.get_logger().trace('ticking {} - {}'
                                                  .format(self.__class__.__name__,
                                                          self.get_status()))
                self.set_status(NodeStatus.RUNNING)
                self.on_tick()
                self._last_ts = current_ts

                # call
                # self.bt.get_logger().info('{} - self._goal_msg: {}'
                #                           .format(self.__class__.__name__, self._goal_msg))
            if(self._goal_msg is not None):
                self._goal_future = self._action_client\
                                        .send_goal_async(self._goal_msg,
                                                         feedback_callback=self.feedback_callback)
                self._goal_future.add_done_callback(self.__goal_response_callback)

    # PUBLIC

    def result_callback(self, future) -> None:
        pass

    def feedback_callback(self, feedback_msg) -> None:
        pass

    def cancel_callback(self) -> None:
        pass
