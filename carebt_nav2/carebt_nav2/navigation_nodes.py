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

from action_msgs.msg import GoalStatus
from carebt.actionNode import ActionNode
from carebt.nodeStatus import NodeStatus
from carebt.parallelNode import ParallelNode
from carebt.rateControlNode import RateControlNode
from carebt_nav2_pyutil.geometry_utils import calculate_path_length, euclidean_distance
from carebt_nav2_pyutil.robot_utils import get_current_pose
from carebt_ros2.rosActionClientActionNode import RosActionClientActionNode
from carebt_ros2.rosActionServerSequenceNode import RosActionServerSequenceNode
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose
from nav2_msgs.action import FollowPath
from nav2_msgs.action import NavigateToPose
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

########################################################################


class CreatePose(ActionNode):

    def __init__(self, bt_runner):
        super().__init__(bt_runner, '?x ?y => ?pose')

    def on_init(self) -> None:
        self._pose = PoseStamped()
        self._pose.pose.position.x = float(self._x)
        self._pose.pose.position.y = float(self._y)
        self._pose.pose.position.z = 0.0
        self._pose.pose.orientation.x = 0.0
        self._pose.pose.orientation.y = 0.0
        self._pose.pose.orientation.z = 0.0
        self._pose.pose.orientation.w = 1.0
        self._pose.header.frame_id = 'map'

        self.set_status(NodeStatus.SUCCESS)

########################################################################


class ComputePathToPoseAction(RosActionClientActionNode):

    def __init__(self, bt_runner):
        super().__init__(bt_runner, ComputePathToPose, 'compute_path_to_pose', '?pose => ?path')

    def on_tick(self) -> None:
        self._goal_msg.goal = self._pose
        self.set_status(NodeStatus.SUSPENDED)

    def result_callback(self, future) -> None:
        status = future.result().status

        if(status == GoalStatus.STATUS_SUCCEEDED):
            self._path = future.result().result.path
            self.set_status(NodeStatus.SUCCESS)
        elif(status == GoalStatus.STATUS_ABORTED):
            # we can not distinguish between an abort due to a new goal (old goal aborted)
            # and a real abort of a goal due to a failure in the processing
            pass

########################################################################


class ComputePathToPoseActionRateLoop(RateControlNode):

    def __init__(self, bt):
        super().__init__(bt, 1000, '?pose => ?path')
        self._pose = None
        self._path = None
        self.set_child(ComputePathToPoseAction, '?pose => ?path')

        self.register_contingency_handler(ComputePathToPoseAction,
                                          [NodeStatus.SUCCESS],
                                          '.*',
                                          self.handle_path_ok)

    def handle_path_ok(self) -> None:
        self.set_current_child_status(NodeStatus.RUNNING)


########################################################################


class FollowPathAction(RosActionClientActionNode):

    def __init__(self, bt_runner):
        super().__init__(bt_runner, FollowPath, 'follow_path', '?path')
        self._current_path = None

    def on_tick(self) -> None:
        self._goal_msg = None
        # if there is a path
        if(self._path is not None):
            # if the path is new
            if(self._current_path != self._path):
                self._goal_msg = FollowPath.Goal()
                self._goal_msg.path = self._path
                self._current_path = self._path

        self.set_status(NodeStatus.RUNNING)

    def on_abort(self) -> None:
        self._goal_handle.cancel_goal()
        print("FollowPathAction - on_abort")

    def result_callback(self, future) -> None:
        status = future.result().status

        if(status == GoalStatus.STATUS_SUCCEEDED):
            self.set_status(NodeStatus.SUCCESS)

        elif(status == GoalStatus.STATUS_ABORTED):
            # currently there is no way to distinguish between an abort due
            # to a new goal (old goal aborted) and a real abort of a goal
            # due to a failure in the processing
            pass

########################################################################


class ApproachPose(ParallelNode):

    def __init__(self, bt_runner):
        super().__init__(bt_runner, 1, '?pose => ?path')
        self._pose = None

    def on_init(self) -> None:
        self.add_child(ComputePathToPoseActionRateLoop, '?pose => ?path')
        self.add_child(FollowPathAction, '?path')

########################################################################


class ApproachPoseSequence(RosActionServerSequenceNode):

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
                                          self.handle_aborted),

    def execute_callback(self, goal_handle):
        self._goal_handle = goal_handle
        self._pose = goal_handle.request.pose
        self.remove_all_children()
        self.set_status(NodeStatus.RUNNING)
        self.append_child(ApproachPose, '?pose => ?path')
        self._start_time = datetime.now()

    def cancel_callback(self, goal_handle):
        print("cancel_callback: {}".format(goal_handle))
        goal_handle.abort()
        goal_handle.destroy()
        self.abort_current_child()

    def handle_goal_reached(self) -> None:
        self.succeed()
        self.remove_all_children()
        self.set_status(NodeStatus.SUSPENDED)

    def handle_aborted(self) -> None:
        self.remove_all_children()
        self.set_status(NodeStatus.SUSPENDED)

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
