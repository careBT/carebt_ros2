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

from threading import Thread


from carebt.abstractLogger import LogLevel
from carebt.behaviorTreeRunner import BehaviorTreeRunner
from carebt_nav2.navigation_nodes import ApproachPoseSequence
from carebt_ros2.plugins.odom_smoother import OdomSmoother
import rclpy
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

########################################################################


class BtNode(Node, Thread):

    def __init__(self):
        Node.__init__(self, 'approach_client')
        Thread.__init__(self)

    def run(self):
        bt_runner = BehaviorTreeRunner()
        bt_runner.get_logger().set_log_level(LogLevel.INFO)
        bt_runner.node = self
        bt_runner.odom_smoother = OdomSmoother(self, 'odom', Duration(nanoseconds=500000000))
        bt_runner.run(ApproachPoseSequence)
        rclpy.shutdown()

########################################################################


def main(args=None):
    rclpy.init(args=args)

    executor = MultiThreadedExecutor()

    btNode = BtNode()
    btNode.start()

    rclpy.spin(btNode, executor=executor)


if __name__ == '__main__':
    main()
