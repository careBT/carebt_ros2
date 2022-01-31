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

from carebt import BehaviorTreeRunner
from carebt import LogLevel
from carebt import TreeNode
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

########################################################################


class BtNode(Thread):

    def __init__(self, ros_node: Node):
        Thread.__init__(self)
        self.__ros_node = ros_node

        # Use a MultiThreadedExecutor to enable processing goals concurrently
        self.__executor = MultiThreadedExecutor()
        self.__bt_runner = BehaviorTreeRunner()

    def thread(self):
        rclpy.spin(self.__ros_node, executor=self.__executor)

    def run_node(self, node: TreeNode, params: str = None):
        self.__bt_runner.get_logger().set_log_level(LogLevel.INFO)
        self.__bt_runner.node = self
        self.__bt_runner.run(node, params)

########################################################################


class RosCarebtRunner(Node):

    def __init__(self):
        rclpy.init(args=None)
        Node.__init__(self, 'carebt_runner')

        self.__btNode = BtNode(self)
        self.__btNode.start()

    def run(self, node: TreeNode, params: str = None) -> None:
        """
        Execute the provided node, respectively the provided behavior tree.

        Parameters
        ----------
        node: TreeNode
            The node which should be executed
        params: str, optional
            The parameters for the node which should be executed

        """
        self.__btNode.run_node(node, params)
