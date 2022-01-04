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

# CREDITS
# The code below is inspired by the OdomSmoother implementation of nav2_util ROS2 package.
# https://github.com/ros-planning/navigation2/blob/main/nav2_util/src/odometry_utils.cpp

from queue import Queue
import threading

from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time


class OdomSmoother:

    def __init__(self,
                 node: Node,
                 odom_topic_name: str,
                 odom_history_duration: Duration):
        node._odom_sub = node.create_subscription(
            Odometry,
            odom_topic_name,
            self._odom_callback,
            10)
        self._odom_cumulate = Odometry()
        self._twist_smoothed = TwistStamped()
        self._history_list = []
        self._odometry_queue = Queue()
        self.odom_history_duration = odom_history_duration

        self._odom_cumulate.twist.twist.linear.x = 0.0
        self._odom_cumulate.twist.twist.linear.y = 0.0
        self._odom_cumulate.twist.twist.linear.z = 0.0
        self._odom_cumulate.twist.twist.angular.x = 0.0
        self._odom_cumulate.twist.twist.angular.y = 0.0
        self._odom_cumulate.twist.twist.angular.z = 0.0

        threading.Thread(target=self._worker).start()

    def _odom_callback(self, msg):
        self._odometry_queue.put(msg)

    def _worker(self):
        while(True):
            odom = self._odometry_queue.get()
            if(len(self._history_list) > 0):
                current_time = Time.from_msg(odom.header.stamp)
                front_time = Time.from_msg(self._history_list[0].header.stamp)
                while (current_time - front_time > self.odom_history_duration):
                    o = self._history_list[0]
                    self._odom_cumulate.twist.twist.linear.x -= o.twist.twist.linear.x
                    self._odom_cumulate.twist.twist.linear.y -= o.twist.twist.linear.y
                    self._odom_cumulate.twist.twist.linear.z -= o.twist.twist.linear.z
                    self._odom_cumulate.twist.twist.angular.x -= o.twist.twist.angular.x
                    self._odom_cumulate.twist.twist.angular.y -= o.twist.twist.angular.y
                    self._odom_cumulate.twist.twist.angular.z -= o.twist.twist.angular.z
                    del self._history_list[0]

                    if(not self._history_list):
                        break

                    front_time = Time.from_msg(self._history_list[0].header.stamp)
            self._history_list.append(odom)
            self._update_state(odom)

    def _update_state(self, odom: Odometry):
        self._odom_cumulate.twist.twist.linear.x += odom.twist.twist.linear.x
        self._odom_cumulate.twist.twist.linear.y += odom.twist.twist.linear.y
        self._odom_cumulate.twist.twist.linear.z += odom.twist.twist.linear.z
        self._odom_cumulate.twist.twist.angular.x += odom.twist.twist.angular.x
        self._odom_cumulate.twist.twist.angular.y += odom.twist.twist.angular.y
        self._odom_cumulate.twist.twist.angular.z += odom.twist.twist.angular.z

        # calculate smoothed twist
        self._twist_smoothed.header = odom.header
        self._twist_smoothed.twist.linear.x =\
            self._odom_cumulate.twist.twist.linear.x / len(self._history_list)
        self._twist_smoothed.twist.linear.y =\
            self._odom_cumulate.twist.twist.linear.y / len(self._history_list)
        self._twist_smoothed.twist.linear.z =\
            self._odom_cumulate.twist.twist.linear.z / len(self._history_list)
        self._twist_smoothed.twist.angular.x =\
            self._odom_cumulate.twist.twist.angular.x / len(self._history_list)
        self._twist_smoothed.twist.angular.y =\
            self._odom_cumulate.twist.twist.angular.y / len(self._history_list)
        self._twist_smoothed.twist.angular.z =\
            self._odom_cumulate.twist.twist.angular.z / len(self._history_list)
        # print('{} {:.2f}'.format(len(self._history_list), self._twist_smoothed.twist.linear.x))

    def get_twist_stamped(self):
        return self._twist_smoothed

    def get_twist(self):
        return self._twist_smoothed.twist
