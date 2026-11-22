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

from typing import TYPE_CHECKING

from carebt.nodeStatus import NodeStatus
from carebt.sequenceNode import SequenceNode
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.task import Future

if TYPE_CHECKING:
    from carebt.behaviorTreeRunner import BehaviorTreeRunner  # pragma: no cover


class RosActionServerSequenceNode(SequenceNode):
    """A `SequenceNode` which provides a ROS 2 action server.

    All callbacks are executed in the thread which spins the ROS node and ticks the
    behavior tree. The execute callback waits for the end of the goal without blocking
    this thread.
    """

    def __init__(self,
                 bt_runner: 'BehaviorTreeRunner',
                 action_type,
                 action_name: str,
                 params: str = None):
        super().__init__(bt_runner, params)
        self.__succeed = False
        self.__terminated = False
        self.__goal_handle = None
        self.__waiters = []
        self.result = action_type.Result()
        self.__action_server = ActionServer(
            bt_runner.node,
            action_type,
            action_name,
            callback_group=ReentrantCallbackGroup(),
            execute_callback=self.__execute_callback,
            goal_callback=self.__goal_callback,
            handle_accepted_callback=self.__handle_accepted_callback,
            cancel_callback=self.__cancel_callback)
        self.set_status(NodeStatus.SUSPENDED)

    def __wake_waiters(self) -> None:
        waiters, self.__waiters = self.__waiters, []
        for waiter in waiters:
            if not waiter.done():
                waiter.set_result(None)

    def __poll_waiters(self) -> None:
        self.__wake_waiters()

    async def __execute_callback(self, goal_handle):
        self.get_logger().debug('{} - __execute_callback'.format(self.__class__.__name__))
        # a goal which was replaced before its execution started is not executed
        if goal_handle is self.__goal_handle:
            self.execute_callback(goal_handle)
        # wait until the goal succeeded, got aborted or was replaced by a new goal
        while(self.__succeed is False
              and self.__terminated is False
              and goal_handle.is_active
              and goal_handle is self.__goal_handle):
            waiter = Future()
            self.__waiters.append(waiter)
            # safety net for goal state changes which do not wake the waiters directly,
            # e.g. a goal handle which is finished by the derived class
            self.create_timer(100, self.__poll_waiters)
            await waiter
        # A replaced goal gets no result, like in Nav2 the client is not informed. Returning
        # would make rclpy send a result, which can crash clients which do not expect the
        # result of the old goal after the response of the new goal, e.g. the Nav2 rviz panel.
        while(goal_handle is not self.__goal_handle and self.__terminated is False):
            waiter = Future()
            self.__waiters.append(waiter)
            await waiter
        if goal_handle.is_active:
            if self.__succeed and goal_handle is self.__goal_handle:
                goal_handle.succeed()
            else:
                goal_handle.abort()
        return self.result

    def __goal_callback(self, goal_request):
        self.get_logger().debug('{} - __goal_callback -- Received goal request: {}'
                                .format(self.__class__.__name__, goal_request))
        return GoalResponse.ACCEPT

    # replace current goal with new goal
    def __handle_accepted_callback(self, goal_handle):
        self.get_logger().debug('{} - __handle_accepted_callback'
                                .format(self.__class__.__name__))
        if self.__goal_handle is not None and self.__goal_handle.is_active:
            self.get_logger().debug('{} - __handle_accepted_callback -- Destroy previous goal'
                                    .format(self.__class__.__name__))
            self.__goal_handle.destroy()
        self.__goal_handle = goal_handle
        self.__succeed = False
        # the execute callback of the replaced goal has to leave its wait loop
        self.__wake_waiters()
        goal_handle.execute()

    def __cancel_callback(self, goal_handle):
        self.get_logger().debug('{} - __cancel_callback'.format(self.__class__.__name__))
        return self.cancel_callback(goal_handle)

    # PROTECTED

    def _internal_on_abort(self) -> None:
        # the timers of the node are canceled, so no execute callback must wait anymore
        self.__terminated = True
        self.__wake_waiters()
        super()._internal_on_abort()

    def _internal_on_delete(self) -> None:
        self.__terminated = True
        self.__wake_waiters()
        super()._internal_on_delete()

    # PUBLIC

    def execute_callback(self, goal_handle: ServerGoalHandle) -> None:
        pass

    def cancel_callback(self, goal_handle: ServerGoalHandle) -> CancelResponse:
        return CancelResponse.ACCEPT

    def succeed(self) -> None:
        self.__succeed = True
        self.__wake_waiters()

    def abort_goal(self) -> None:
        """Abort the current goal, if it is still active, so that the client gets a result."""
        if self.__goal_handle is not None and self.__goal_handle.is_active:
            self.__goal_handle.abort()
        self.__wake_waiters()
