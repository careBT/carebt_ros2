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

from threading import Thread
from typing import TYPE_CHECKING

from carebt.actionNode import ActionNode
from carebt.nodeStatus import NodeStatus


if TYPE_CHECKING:
    from carebt.behaviorTreeRunner import BehaviorTreeRunner  # pragma: no cover


class RosServiceClientActionNode(ActionNode):
    """Call a ROS2 service.

    Generic BT action node that issues a single request/response service
    call.  The underlying ``rclpy`` client is obtained from the shared
    :class:`carebt_ros2.RosClientManager` attached to the BT runner's ROS
    node, so repeated instantiations reuse the same client instead of
    leaking a new one on every BT tick.

    Input Parameters
    ----------------
    ?service : str
        The ROS2 service name
    ?type
        The service type class
    ?request
        The service request message

    Output Parameters
    -----------------
    ?response
        The service response message
    """

    def __init__(self, bt_runner: 'BehaviorTreeRunner'):
        super().__init__(bt_runner, '?service ?type ?request => ?response')
        self.__bt_runner = bt_runner

    def on_init(self) -> None:
        self.get_logger().info('{} - call service {} with {}'
                               .format(self.__class__.__name__,
                                       self._service,
                                       self._type))
        self.set_status(NodeStatus.SUSPENDED)
        Thread(target=self.__worker, daemon=True).start()

    def __worker(self) -> None:
        client = self.__bt_runner.node.client_manager \
            .get_or_create_service_client(self._type, self._service)
        if client.wait_for_service(timeout_sec=1.0):
            self._response = client.call(self._request)
            self.set_status(NodeStatus.SUCCESS)
            self.get_logger().info(
                f'service call successful')
        else:
            self.set_status(NodeStatus.FAILURE)
            self.set_contingency_message('SERVICE_NOT_AVAILABLE')
            self.get_logger().warn(f'service not available: {self._service}')
