# Copyright 2026 Andreas Steck (steck.andi@gmail.com)
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

from time import monotonic
from typing import Callable, Optional, TYPE_CHECKING

from carebt.actionNode import ActionNode
from carebt.nodeStatus import NodeStatus
from rclpy.client import Client
from rclpy.task import Future

if TYPE_CHECKING:
    from carebt.behaviorTreeRunner import BehaviorTreeRunner  # pragma: no cover

_POLL_MS = 100


class RosServiceCallActionNode(ActionNode):
    """Base class for action nodes which call ROS 2 services without blocking.

    `call_service` waits for the service with careBT timers and sends the request with
    `call_async`. All callbacks are executed in the thread which spins the ROS node and
    ticks the behavior tree, so they can safely change the state of the node. The node
    must be in status `SUSPENDED` while the call is pending.
    """

    def __init__(self, bt_runner: 'BehaviorTreeRunner', params: str = None):
        super().__init__(bt_runner, params)

    def call_service(self,
                     client: Client,
                     request,
                     response_callback: Callable[[object], None],
                     wait_ms: Optional[int] = 1000) -> None:
        """Send the request as soon as the service is available.

        Parameters
        ----------
        client: Client
            The service client
        request
            The service request
        response_callback: Callable
            Called with the service response. Not called if the node is not
            `SUSPENDED` anymore, e.g. due to a timeout or an abort.
        wait_ms: int, optional
            Time to wait for the service. After that the node fails with the
            contingency message `SERVICE_NOT_AVAILABLE`. None waits until the
            node is not `SUSPENDED` anymore, e.g. due to its timeout.

        """
        deadline = None if wait_ms is None else monotonic() + wait_ms / 1000
        self.__poll(client, request, response_callback, deadline)

    # PRIVATE

    def __poll(self, client, request, response_callback, deadline) -> None:
        if(self.get_status() != NodeStatus.SUSPENDED):
            return
        if(client.service_is_ready()):
            future = client.call_async(request)
            future.add_done_callback(
                lambda f: self.__on_response(f, response_callback))
        elif(deadline is not None and monotonic() >= deadline):
            self.get_logger().warn(f'{self.__class__.__name__} - service not available: '
                                   f'{client.srv_name}')
            self.set_status(NodeStatus.FAILURE)
            self.set_contingency_message('SERVICE_NOT_AVAILABLE')
        else:
            self.create_timer(_POLL_MS,
                              lambda: self.__poll(client, request, response_callback, deadline))

    def __on_response(self, future: Future, response_callback: Callable) -> None:
        if(self.get_status() != NodeStatus.SUSPENDED):
            return
        if(future.exception() is not None):
            self.get_logger().warn(f'{self.__class__.__name__} - service call failed: '
                                   f'{future.exception()}')
            self.set_status(NodeStatus.FAILURE)
            self.set_contingency_message('SERVICE_CALL_FAILED')
            return
        response_callback(future.result())
