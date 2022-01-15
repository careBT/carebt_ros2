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
import math
from threading import Thread

from carebt.abstractLogger import LogLevel
from carebt.nodeStatus import NodeStatus
from carebt.behaviorTreeRunner import BehaviorTreeRunner
from carebt_nav2.navigation_nodes import ApproachPose
from carebt_nav2_pyutil.geometry_utils import calculate_path_length, euclidean_distance
from carebt_nav2_pyutil.robot_utils import get_current_pose
from carebt_ros2.rosActionServerSequenceNode import RosActionServerSequenceNode
from carebt_ros2.plugins.odom_smoother import OdomSmoother
import rclpy
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

########################################################################


class ApproachPoseSequence(RosActionServerSequenceNode):
    """The `ApproachPoseSequence` provides the ROS2 ActionServer for the NavigateToPose.

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
        self._start_time = None
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, bt_runner.node)
        self._odom_smoother = bt_runner.odom_smoother

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
        self.append_child(ApproachPose, '?pose => ?path')
        self._start_time = datetime.now()
        self.get_logger().info('{} - starting to approach the goal (x, y): ({:.2f}, {:.2f})'
                               .format(self.__class__.__name__,
                                       self._pose.pose.position.x,
                                       self._pose.pose.position.y))

    def cancel_callback(self, goal_handle):
        goal_handle.abort()
        goal_handle.destroy()
        self.abort_current_child()
        self.get_logger().info('{} - goal canceled.'.format(self.__class__.__name__))

    def handle_goal_reached(self) -> None:
        self.succeed()
        self.remove_all_children()
        self.set_status(NodeStatus.SUSPENDED)
        self.get_logger().info('{} - goal reached. Waiting for new goal...'.format(self.__class__.__name__))

    def handle_aborted(self) -> None:
        self.remove_all_children()
        self.set_status(NodeStatus.SUSPENDED)
        self.get_logger().info('{} - goal aborted. Waiting for new goal...'.format(self.__class__.__name__))

    def on_tick(self) -> None:
        feedback_msg = NavigateToPose.Feedback()

        # get current pose
        robot_frame = 'base_link'
        global_frame = 'map'
        current_pose = get_current_pose(global_frame,
                                        robot_frame,
                                        self._tf_buffer)

        # Find the closest pose on global path and calculate the path length
        if(self._path is not None):
            closest_pose_idx = 0
            curr_min_dist = float('inf')
            for idx, path_pose in enumerate(self._path.poses):
                curr_dist = euclidean_distance(current_pose.pose, path_pose.pose)
                if (curr_dist < curr_min_dist):
                    curr_min_dist = curr_dist
                    closest_pose_idx = idx
            # Calculate path length
            feedback_msg.distance_remaining = calculate_path_length(self._path, closest_pose_idx)

        # navigation_time
        delta = (datetime.now() - self._start_time).total_seconds()
        feedback_msg.navigation_time.sec = int(delta)

        # estimated_time_remaining
        twist = self._odom_smoother.get_twist()
        current_linear_speed = math.hypot(twist.linear.x, twist.linear.y)
        # Calculate estimated time taken to goal if speed is higher than 1cm/s
        # and at least 10cm to go
        if(abs(current_linear_speed) > 0.01 and feedback_msg.distance_remaining > 0.1):
            feedback_msg.estimated_time_remaining.sec =\
                int(feedback_msg.distance_remaining / abs(current_linear_speed))
        else:
            feedback_msg.estimated_time_remaining.sec = 0

        # number_of_recoveries TODO
        feedback_msg.number_of_recoveries = 0  # TODO

        # send feedback
        self._goal_handle.publish_feedback(feedback_msg)

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
