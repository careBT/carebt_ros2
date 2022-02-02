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

from action_msgs.msg import GoalStatus
from carebt.actionNode import ActionNode
from carebt.nodeStatus import NodeStatus
from carebt.parallelNode import ParallelNode
from carebt.rateControlNode import RateControlNode
from carebt_nav2_pyutil.geometry_utils import calculate_remaining_path_length
from carebt_nav2_pyutil.geometry_utils import calculate_travel_time
from carebt_nav2_pyutil.robot_utils import get_current_pose
from carebt_ros2.rosActionClientActionNode import RosActionClientActionNode
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.action import ComputePathThroughPoses
from nav2_msgs.action import ComputePathToPose
from nav2_msgs.action import FollowPath
from nav2_msgs.action import NavigateToPose
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

########################################################################


class InitPoseAction(ActionNode):
    """Publishes the current pose of the robot.

    Publishes the current pose of the robot to the '/initialpose' topic. This pose is used
    by the localization component.

    Input Parameters
    ----------------
    ?pose : geometry_msgs.msg.PoseWithCovarianceStamped
        The current pose
    """

    def __init__(self, bt_runner):
        super().__init__(bt_runner, '?initial_pose')
        self._initial_pose = PoseWithCovarianceStamped()
        self._initialpose_pub = bt_runner.node.create_publisher(PoseWithCovarianceStamped,
                                                                '/initialpose',
                                                                10)

    def on_tick(self) -> None:
        self._initialpose_pub.publish(self._initial_pose)

        self.set_status(NodeStatus.SUCCESS)

########################################################################


class ComputePathToPoseAction(RosActionClientActionNode):
    """Provides a path to the goal position.

    Input Parameters
    ----------------
    ?start : geometry_msgs.msg.PoseStamped
        The optional start position

    ?goal : geometry_msgs.msg.PoseStamped
        The goal position

    Output Parameters
    -----------------
    ?path : nav_msgs.msg.Path
        The planned path
    """

    def __init__(self, bt_runner):
        super().__init__(bt_runner, ComputePathToPose, 'compute_path_to_pose',
                         '?start ?goal => ?path')

    def on_tick(self) -> None:
        if(self._start is not None):
            self._goal_msg.start = self._start
            self._goal_msg.use_start = True
        else:
            self._goal_msg.use_start = False
        self._goal_msg.goal = self._goal
        self._goal_msg.planner_id = ''  # TODO: select planner from kb
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


class ComputePathThroughPosesAction(RosActionClientActionNode):
    """Provides a path to the goal position through intermediate waypoints.

    Input Parameters
    ----------------
    ?start : geometry_msgs.msg.PoseStamped
        The optional start position

    ?goals : geometry_msgs.msg.PoseStamped[]
        The goal position with intermediate waypoints

    Output Parameters
    -----------------
    ?path : nav_msgs.msg.Path
        The planned path
    """

    def __init__(self, bt_runner):
        super().__init__(bt_runner, ComputePathThroughPoses,
                         'compute_path_through_poses', '?start ?goals => ?path')

    def on_tick(self) -> None:
        if(self._start is not None):
            self._goal_msg.start = self._start
            self._goal_msg.use_start = True
        else:
            self._goal_msg.use_start = False
        self._goal_msg.goals = self._goals
        self._goal_msg.planner_id = ''  # TODO: select planner from kb
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
    """Makes the `ComputePathToPoseAction` node running in a loop.

    The `ComputePathToPoseActionRateLoop` makes the `ComputePathToPoseAction` node running in
    a loop. As soon as the `ComputePathToPoseAction` node returns with `SUCCESS` the status
    is set back to `RUNNING`.

    Input Parameters
    ----------------
    ?start : geometry_msgs.msg.PoseStamped
        The optional start position

    ?goal : geometry_msgs.msg.PoseStamped
        The goal position

    Output Parameters
    -----------------
    ?path : nav_msgs.msg.Path
        The planned path
    """

    def __init__(self, bt):
        super().__init__(bt, 1000, '?start ?goal => ?path')
        self._pose = None
        self._path = None
        self.set_child(ComputePathToPoseAction, '?start ?goal => ?path')

        self.register_contingency_handler(ComputePathToPoseAction,
                                          [NodeStatus.SUCCESS],
                                          '.*',
                                          self.handle_path_ok)

    def handle_path_ok(self) -> None:
        self.set_current_child_status(NodeStatus.RUNNING)


########################################################################


class ComputePathThroughPosesActionRateLoop(RateControlNode):
    """Makes the `ComputePathThroughPosesAction` node running in a loop.

    The `ComputePathThroughPosesActionRateLoop` makes the `ComputePathThroughPosesAction`
    node running in a loop. As soon as the `ComputePathToPoseAction` node returns with
    `SUCCESS` the status is set back to `RUNNING`.

    Input Parameters
    ----------------
    ?start : geometry_msgs.msg.PoseStamped
        The optional start position

    ?goals : geometry_msgs.msg.PoseStamped[]
        The goal position with intermediate waypoints

    Output Parameters
    -----------------
    ?path : nav_msgs.msg.Path
        The planned path
    """

    def __init__(self, bt):
        super().__init__(bt, 1000, '?start ?goals => ?path')
        self._poses = None
        self._path = None
        self.set_child(ComputePathThroughPosesAction, '?start ?goals => ?path')

        self.register_contingency_handler(ComputePathThroughPosesAction,
                                          [NodeStatus.SUCCESS],
                                          '.*',
                                          self.handle_path_ok)

    def handle_path_ok(self) -> None:
        self.set_current_child_status(NodeStatus.RUNNING)

########################################################################


class CreateFollowPathFeedback(ActionNode):
    """Returns feedback for the current NavigateToPose action.

    Input Parameters
    ----------------
    ?path : nav_msgs.msg.Path
        The planned path

    Output Parameters
    -----------------
    ?feedback : nav2_msgs.action.NavigateToPose_FeedbackMessage
        The feedback for the current NavigateToPose action
    """

    def __init__(self, bt_runner):
        super().__init__(bt_runner, '?path => ?feedback')
        self._feedback = NavigateToPose.Feedback()
        self._start_time = datetime.now()
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, bt_runner.node)
        self._odom_smoother = bt_runner.odom_smoother

    def on_tick(self) -> None:
        # get current pose
        robot_frame = 'base_link'
        global_frame = 'map'
        current_pose = get_current_pose(global_frame,
                                        robot_frame,
                                        self._tf_buffer)
        if(current_pose is not None):
            self._feedback.current_pose = current_pose

        # remaining path length
        if(self._path is not None and current_pose is not None):
            self._feedback.distance_remaining =\
                calculate_remaining_path_length(self._path, current_pose)

        # navigation_time since start
        delta = (datetime.now() - self._start_time).total_seconds()
        self._feedback.navigation_time.sec = int(delta)

        # estimated_time_remaining
        twist = self._odom_smoother.get_twist()
        self._feedback.estimated_time_remaining.sec =\
            calculate_travel_time(twist, self._feedback.distance_remaining)

        # number_of_recoveries TODO
        self._feedback.number_of_recoveries = 0  # TODO

########################################################################


class FollowPathAction(RosActionClientActionNode):
    """Send the path to the ROS2 controller node.

    Input Parameters
    ----------------
    ?path : nav_msgs.msg.Path
        The planned path to follow
    """

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
        self.get_logger().info('{} - aborting...'.format(self.__class__.__name__))
        self.get_goal_handle().cancel_goal()

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
    """Approch a goal position.

    It runs the two child nodes to compute the path and to follow this path in
    parallel. The current path is provided as output parameter, that it can be
    'analysed' while the node is executing by a parent node.

    Input Parameters
    ----------------
    ?goal : geometry_msgs.msg.PoseStamped
        The goal position

    Output Parameters
    -----------------
    ?feedback : nav2_msgs.action.NavigateToPose_FeedbackMessage
        The feedback for the current NavigateToPose action
    """

    def __init__(self, bt_runner):
        super().__init__(bt_runner, 1, '?goal => ?feedback')
        self._pose = None

    def on_init(self) -> None:
        self.add_child(ComputePathToPoseActionRateLoop, 'None ?goal => ?path')
        self.add_child(FollowPathAction, '?path')
        self.add_child(CreateFollowPathFeedback, '?path => ?feedback')

########################################################################


class ApproachPoseThroughPoses(ParallelNode):
    """Approch a goal position through intermediate waypoints.

    It runs the two child nodes to compute the path and to follow this path in
    parallel. The current path is provided as output parameter, that it can be
    'analysed' while the node is executing by a parent node.

    Input Parameters
    ----------------
    ?goals : geometry_msgs.msg.PoseStamped[]
        The goal position with intermediate waypoints

    Output Parameters
    -----------------
    ?feedback : nav2_msgs.action.NavigateToPose_FeedbackMessage
        The feedback for the current NavigateToPose action
    """

    def __init__(self, bt_runner):
        super().__init__(bt_runner, 1, '?goals => ?feedback')
        self._poses = None

    def on_init(self) -> None:
        self.add_child(ComputePathThroughPosesActionRateLoop, 'None ?goals => ?path')
        self.add_child(FollowPathAction, '?path')
        self.add_child(CreateFollowPathFeedback, '?path => ?feedback')
