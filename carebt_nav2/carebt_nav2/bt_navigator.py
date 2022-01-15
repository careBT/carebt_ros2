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
from threading import Thread

from carebt.abstractLogger import LogLevel
from carebt.behaviorTreeRunner import BehaviorTreeRunner
from carebt.nodeStatus import NodeStatus
from carebt.parallelNode import ParallelNode
from carebt_nav2.navigation_nodes import ApproachPose
from carebt_nav2.navigation_nodes import ApproachPoseThroughPoses
from carebt_nav2_pyutil.geometry_utils import euclidean_distance
from carebt_nav2_pyutil.robot_utils import get_current_pose
from carebt_ros2.plugins.odom_smoother import OdomSmoother
from carebt_ros2.rosActionServerSequenceNode import RosActionServerSequenceNode
from nav2_msgs.action import NavigateThroughPoses
from nav2_msgs.action import NavigateToPose
import rclpy
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

########################################################################


class ApproachPoseSequence(RosActionServerSequenceNode):
    """ROS2 ActionServer for the NavigateToPose action.

    After the node is initialized it is in state `SUSPENDED` and waits for a pose
    to navigate to. As soon as a request arrives the `execute_callback` is called which
    adds the `ApproachPose` as child and sets the state of the node to `RUNNING`. If the
    `ApproachPose` node returns with `SUCCESS` the `ApproachPoseSequence` is set to status
    `SUSPENDED`. In the `on_tick` method the feedback is calculated.
    """

    def __init__(self, bt_runner):
        super().__init__(bt_runner, NavigateToPose, 'navigate_to_pose')
        self.set_throttle_ms(250)
        self._goal_handle = None

    def on_init(self) -> None:
        self.register_contingency_handler(ApproachPose,
                                          [NodeStatus.SUCCESS],
                                          r'.*',
                                          self.handle_goal_reached)
        self.register_contingency_handler(ApproachPose,
                                          [NodeStatus.ABORTED],
                                          r'.*',
                                          self.handle_aborted)
        self.get_logger().info('{} - waiting for goal...'.format(self.__class__.__name__))

    def execute_callback(self, goal_handle):
        self._goal_handle = goal_handle
        self._pose = goal_handle.request.pose
        self.remove_all_children()
        self.set_status(NodeStatus.RUNNING)
        self.append_child(ApproachPose, '?pose => ?feedback')
        self._start_time = datetime.now()
        self.get_logger().info('{} - starting to approach the goal (x, y): ({:.2f}, {:.2f})'
                               .format(self.__class__.__name__,
                                       self._pose.pose.position.x,
                                       self._pose.pose.position.y))

    def cancel_callback(self, goal_handle):
        goal_handle.abort()
        goal_handle.destroy()
        self.abort_current_child()
        self.get_logger().info('{} - goal canceled'.format(self.__class__.__name__))

    def handle_goal_reached(self) -> None:
        self.succeed()
        self.remove_all_children()
        self.set_status(NodeStatus.SUSPENDED)
        self.get_logger().info('{} - goal reached. Waiting for new goal...'
                               .format(self.__class__.__name__))

    def handle_aborted(self) -> None:
        self.remove_all_children()
        self.set_status(NodeStatus.SUSPENDED)
        self.get_logger().info('{} - goal aborted. Waiting for new goal...'
                               .format(self.__class__.__name__))

    def on_tick(self) -> None:
        # send feedback
        self._goal_handle.publish_feedback(self._feedback)

########################################################################


class ApproachPoseThroughPosesSequence(RosActionServerSequenceNode):
    """ROS2 ActionServer for the NavigateThroughPoses action.

    After the node is initialized it is in state `SUSPENDED` and waits for the poses
    to navigate to. As soon as a request arrives the `execute_callback` is called which
    adds the `ApproachPoseThroughPoses` as child and sets the state of the node to `RUNNING`.
    If the `ApproachPoseThroughPoses` node returns with `SUCCESS` the
    `ApproachPoseThroughPosesSequence` is set to status `SUSPENDED`. In the `on_tick`
    method the feedback is calculated.
    """

    def __init__(self, bt_runner):
        super().__init__(bt_runner, NavigateThroughPoses, 'navigate_through_poses')
        self.set_throttle_ms(250)
        self._goal_handle = None
        self._start_time = None
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, bt_runner.node)
        self._odom_smoother = bt_runner.odom_smoother

    def on_init(self) -> None:
        self.register_contingency_handler(ApproachPoseThroughPoses,
                                          [NodeStatus.SUCCESS],
                                          r'.*',
                                          self.handle_goal_reached)
        self.register_contingency_handler(ApproachPoseThroughPoses,
                                          [NodeStatus.ABORTED],
                                          r'.*',
                                          self.handle_aborted)
        self.get_logger().info('{} - waiting for goals...'.format(self.__class__.__name__))

    def execute_callback(self, goal_handle):
        self._goal_handle = goal_handle
        self._poses = goal_handle.request.poses
        self.remove_all_children()
        self.set_status(NodeStatus.RUNNING)
        self.append_child(ApproachPoseThroughPoses, '?poses => ?feedback')
        self._start_time = datetime.now()
        self.get_logger().info('{} - starting to approach the goal (x, y): ({:.2f}, {:.2f}) '
                               'through {} waypoints'
                               .format(self.__class__.__name__,
                                       self._poses[-1].pose.position.x,
                                       self._poses[-1].pose.position.y,
                                       len(self._poses)-1))

    def cancel_callback(self, goal_handle):
        goal_handle.abort()
        goal_handle.destroy()
        self.abort_current_child()
        self.get_logger().info('{} - goal canceled'.format(self.__class__.__name__))

    def handle_goal_reached(self) -> None:
        self.succeed()
        self.remove_all_children()
        self.set_status(NodeStatus.SUSPENDED)
        self.get_logger().info('{} - goal reached. Waiting for new goals...'
                               .format(self.__class__.__name__))

    def handle_aborted(self) -> None:
        self.remove_all_children()
        self.set_status(NodeStatus.SUSPENDED)
        self.get_logger().info('{} - goal aborted. Waiting for new goals...'
                               .format(self.__class__.__name__))

    def on_tick(self) -> None:
        feedback_msg = NavigateThroughPoses.Feedback()

        # get current pose
        robot_frame = 'base_link'
        global_frame = 'map'
        current_pose = get_current_pose(global_frame,
                                        robot_frame,
                                        self._tf_buffer)

        # check if next intermediate waypoint has been passed
        # TODO remove magic number
        if(len(self._poses) > 0
           and euclidean_distance(current_pose.pose, self._poses[0].pose) < 0.5):
            del self._poses[0]
        # set number of remaining poses
        feedback_msg.number_of_poses_remaining = len(self._poses)

        feedback_msg.current_pose = self._feedback.current_pose
        feedback_msg.distance_remaining = self._feedback.distance_remaining
        feedback_msg.navigation_time.sec = self._feedback.navigation_time.sec
        feedback_msg.estimated_time_remaining.sec = self._feedback.estimated_time_remaining.sec
        feedback_msg.number_of_recoveries = self._feedback.number_of_recoveries

        # send feedback
        self._goal_handle.publish_feedback(feedback_msg)


########################################################################

class NavigatorNode(ParallelNode):
    """Navigate to poses or through poses."""

    def __init__(self, bt_runner):
        super().__init__(bt_runner, 2, '')
        self._pose = None

    def on_init(self) -> None:
        self.add_child(ApproachPoseSequence)
        self.add_child(ApproachPoseThroughPosesSequence)

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
        bt_runner.run(NavigatorNode)
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
