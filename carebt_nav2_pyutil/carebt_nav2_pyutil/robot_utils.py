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

from geometry_msgs.msg import PoseStamped
import rclpy
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer


def get_current_pose(global_frame: str,
                     robot_frame: str,
                     tf: Buffer) -> PoseStamped:
    """
    Get the current pose of the robot.

    Parameters
    ----------
    global_frame: str
        Frame to transform into (e.g. map)
    robot_frame: str
        Frame to transform from (e.g. base_link)
    tf: Buffer
        TF buffer to use for the transformation

    Returns
    -------
    PoseStamped
        Current pose of the robot.

    """
    trans = None
    now = rclpy.time.Time()
    try:
        trans = tf.lookup_transform(
            global_frame,
            robot_frame,
            now,
            rclpy.duration.Duration(seconds=0.2))
    except TransformException:
        return
    current_pose = PoseStamped()
    current_pose.pose.position.x = trans.transform.translation.x
    current_pose.pose.position.y = trans.transform.translation.y
    current_pose.pose.orientation.x = trans.transform.rotation.x
    current_pose.pose.orientation.y = trans.transform.rotation.y
    current_pose.pose.orientation.z = trans.transform.rotation.z
    current_pose.pose.orientation.w = trans.transform.rotation.w
    current_pose.header.stamp = now.to_msg()
    current_pose.header.frame_id = global_frame
    return current_pose
