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

from action_msgs.msg import GoalStatus
from carebt.actionNode import ActionNode
from carebt.nodeStatus import NodeStatus
from carebt.parallelNode import ParallelNode
from carebt.rateControlNode import RateControlNode
from carebt_ros2.rosActionClientActionNode import RosActionClientActionNode
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathThroughPoses, ComputePathToPose
from nav2_msgs.action import FollowPath

########################################################################


class CreatePose(ActionNode):
    """Returns a PoseStamped.

    The `CreatePose` node returns a PoseStamped corresponding to
    the provided ?x and ?y coordinates. This node is just for
    testing purposes.

    Input Parameters
    ----------------
    ?x : int
        The x coordinate
    ?y : int
        The y coordinate

    Output Parameters
    -----------------
    ?pose : geometry_msgs.msg.PoseStamped
        The pose corresponding to ?x and ?y
    """

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
    """Provides a path to the goal position.

    Input Parameters
    ----------------
    ?pose : PoseStamped
        The goal position

    Output Parameters
    -----------------
    ?path : nav_msgs.msg.Path
        The planned path
    """

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
    ?poses : PoseStamped[]
        The goal position with intermediate waypoints

    Output Parameters
    -----------------
    ?path : nav_msgs.msg.Path
        The planned path
    """

    def __init__(self, bt_runner):
        super().__init__(bt_runner,
                         ComputePathThroughPoses,
                        'compute_path_through_poses',
                        '?poses => ?path')

    def on_tick(self) -> None:
        self._goal_msg.goals = self._poses
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
    ?pose : PoseStamped
        The goal position

    Output Parameters
    -----------------
    ?path : nav_msgs.msg.Path
        The planned path
    """

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


class ComputePathThroughPosesActionRateLoop(RateControlNode):
    """Makes the `ComputePathThroughPosesAction` node running in a loop.

    The `ComputePathThroughPosesActionRateLoop` makes the `ComputePathThroughPosesAction`
    node running in a loop. As soon as the `ComputePathToPoseAction` node returns with
    `SUCCESS` the status is set back to `RUNNING`.

    Input Parameters
    ----------------
    ?poses : PoseStamped[]
        The goal position with intermediate waypoints

    Output Parameters
    -----------------
    ?path : nav_msgs.msg.Path
        The planned path
    """

    def __init__(self, bt):
        super().__init__(bt, 1000, '?poses => ?path')
        self._poses = None
        self._path = None
        self.set_child(ComputePathThroughPosesAction, '?poses => ?path')

        self.register_contingency_handler(ComputePathThroughPosesAction,
                                          [NodeStatus.SUCCESS],
                                          '.*',
                                          self.handle_path_ok)

    def handle_path_ok(self) -> None:
        self.set_current_child_status(NodeStatus.RUNNING)

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
    ?pose : PoseStamped
        The goal position

    Output Parameters
    -----------------
    ?path : nav_msgs.msg.Path
        The planned path
    """

    def __init__(self, bt_runner):
        super().__init__(bt_runner, 1, '?pose => ?path')
        self._pose = None

    def on_init(self) -> None:
        self.add_child(ComputePathToPoseActionRateLoop, '?pose => ?path')
        self.add_child(FollowPathAction, '?path')

########################################################################


class ApproachPoseThroughPoses(ParallelNode):
    """Approch a goal position through intermediate waypoints.

    It runs the two child nodes to compute the path and to follow this path in
    parallel. The current path is provided as output parameter, that it can be
    'analysed' while the node is executing by a parent node.

    Input Parameters
    ----------------
    ?poses : PoseStamped[]
        The goal position with intermediate waypoints

    Output Parameters
    -----------------
    ?path : nav_msgs.msg.Path
        The planned path
    """

    def __init__(self, bt_runner):
        super().__init__(bt_runner, 1, '?poses => ?path')
        self._poses = None

    def on_init(self) -> None:
        self.add_child(ComputePathThroughPosesActionRateLoop, '?poses => ?path')
        self.add_child(FollowPathAction, '?path')
