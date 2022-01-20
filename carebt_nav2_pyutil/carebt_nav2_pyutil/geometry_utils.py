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

import math

from geometry_msgs.msg import Pose, PoseStamped, Twist
from nav_msgs.msg import Path


def euclidean_distance(pos1: Pose, pos2: Pose) -> float:
    """Get the L2 distance between 2 geometry_msgs::Poses.

    Parameters
    ----------
    pos1: geometry_msgs.msg.Pose
        First pose
    pos2: geometry_msgs.msg.Pose
        Second pose

    Returns
    -------
    float
        L2 distance
    """
    dx = pos1.position.x - pos2.position.x
    dy = pos1.position.y - pos2.position.y
    dz = pos1.position.z - pos2.position.z
    return math.sqrt(dx * dx + dy * dy + dz * dz)


def calculate_path_length(path: Path, start_index: int = 0) -> float:
    """Calculate the length of the provided path, starting at the provided index.

    Parameters
    ----------
    path: nav_msgs.msg.Path
        The planned path with the poses
    start_index: int = 0
        Optional argument specifying the starting index for the calculation of path length.
        Provide this if you want to calculate length of a subset of the path.

    Returns
    -------
    float
        Path length
    """
    if(start_index + 1 >= len(path.poses)):
        return 0.0
    path_length = 0.0
    for idx in range(start_index, len(path.poses) - 1):
        path_length += euclidean_distance(path.poses[idx].pose, path.poses[idx+1].pose)
    return path_length


def calculate_remaining_path_length(path: Path, pose: PoseStamped) -> float:
    """Calculate the length of the provided path, starting at the provided pose.

    Parameters
    ----------
    path: nav_msgs.msg.Path
        The planned path with the poses
    pose: geometry_msgs.msg.PoseStamped
        The pose to calculate the remaining path length from

    Returns
    -------
    float
        Path length
    """
    closest_pose_idx = 0
    for idx, path_pose in enumerate(path.poses):
        curr_dist = euclidean_distance(pose.pose, path_pose.pose)
        # TODO remove magic number 0.5
        if (curr_dist < 0.5):
            closest_pose_idx = idx
            break
    return calculate_path_length(path, closest_pose_idx)


def calculate_travel_time(twist: Twist, path_length: float) -> int:
    """Calculate the time to travel the path given the provided speed (twist).

    Parameters
    ----------
    twist: geometry_msgs.msg.Twist
        The 'estimated' twist'
    path_length: float
        The path length

    Returns
    -------
    int
        Time
    """
    time = 0
    current_linear_speed = math.hypot(twist.linear.x, twist.linear.y)
    # Calculate estimated time taken to goal if speed is higher than 1cm/s
    # and at least 10cm to go
    if(abs(current_linear_speed) > 0.01 and path_length > 0.1):
        time = int(path_length / abs(current_linear_speed))
    return time
