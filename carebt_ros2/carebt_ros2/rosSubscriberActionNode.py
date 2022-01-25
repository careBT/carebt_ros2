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

from typing import TYPE_CHECKING

from carebt.actionNode import ActionNode
from carebt.nodeStatus import NodeStatus


if TYPE_CHECKING:
    from carebt.behaviorTreeRunner import BehaviorTreeRunner  # pragma: no cover


class RosSubscriberActionNode(ActionNode):

    def __init__(self,
                 bt_runner: 'BehaviorTreeRunner',
                 topic_type,
                 topic_name: str,
                 params: str = None):
        super().__init__(bt_runner, params)
        self.__subscriber = bt_runner.node.create_subscription(
            topic_type,
            topic_name,
            self.__topic_callback,
            10)
        self.set_status(NodeStatus.SUSPENDED)

    def __topic_callback(self, msg):
        self.topic_callback(msg)

    # PUBLIC

    def topic_callback(self, msg) -> None:
        pass
