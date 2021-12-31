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

from geometry_msgs.msg import Pose
from nav_msgs.msg import Path


def euclidean_distance(pos1: Pose, pos2: Pose) -> float:
    """Get the L2 distance between 2 geometry_msgs::Poses.

    Parameters
    ----------
    pos1: Pose
        First pose
    pos2: Pose
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
    path: Path
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
