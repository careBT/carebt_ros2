# Copyright 2022 Andreas Steck (steck.andi@gmail.com)
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

from carebt_kb.plugin_base import PluginBase
from datetime import datetime
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.time import Time
from rclpy.clock import Clock
from rclpy_message_converter import message_converter


class OdomSmoother(PluginBase):

    def on_init_callback(self, plugin_name: str):
        self.__last_kb_update = datetime.min

        self._kb_server.declare_parameter(f'{plugin_name}.topic', 'odom')
        self._kb_server.declare_parameter(f'{plugin_name}.history_duration_ms', 10000)
        self._kb_server.declare_parameter(f'{plugin_name}.slot', '')
        self._kb_server.declare_parameter(f'{plugin_name}.kb_filter', '')
        self._kb_server.declare_parameter(f'{plugin_name}.kb_update_rate_ms', 1000)

        topic = self._kb_server.get_parameter(
                f'{plugin_name}.topic').get_parameter_value().string_value
        history_duration_ms = self._kb_server.get_parameter(
                f'{plugin_name}.history_duration_ms').get_parameter_value().integer_value
        self.__slot = self._kb_server.get_parameter(
                f'{plugin_name}.slot').get_parameter_value().string_value
        self.__kb_filter = self._kb_server.get_parameter(
                f'{plugin_name}.kb_filter').get_parameter_value().string_value
        self.__kb_update_rate_ms = self._kb_server.get_parameter(
                f'{plugin_name}.kb_update_rate_ms').get_parameter_value().integer_value

        self._kb_server.get_logger().info(f'OdomSmoother - topic= {topic}')
        self._kb_server.get_logger().info(f'OdomSmoother - slot= {self.__slot}')
        self._kb_server.get_logger().info(f'OdomSmoother - history_duration_ms= {history_duration_ms}')
        self._kb_server.get_logger().info(f'OdomSmoother - kb_update_rate_ms= {self.__kb_update_rate_ms}')

        self.__odom_cumulate = Odometry()
        self.__twist_smoothed = TwistStamped()
        self.__history_list = []
        self.__odometry_queue = Queue()
        self.__odom_history_duration = Duration(nanoseconds=history_duration_ms * 1000000)

        self.__odom_cumulate.twist.twist.linear.x = 0.0
        self.__odom_cumulate.twist.twist.linear.y = 0.0
        self.__odom_cumulate.twist.twist.linear.z = 0.0
        self.__odom_cumulate.twist.twist.angular.x = 0.0
        self.__odom_cumulate.twist.twist.angular.y = 0.0
        self.__odom_cumulate.twist.twist.angular.z = 0.0

        self._kb_server.create_subscription(
            Odometry,
            topic,
            self.__odom_callback,
            10)

        threading.Thread(target=self.__worker, daemon=True).start()


    def __odom_callback(self, msg):
        self.__odometry_queue.put(msg)

    def __worker(self):
        while(True):
            odom = self.__odometry_queue.get()
            if(len(self.__history_list) > 0):
                current_time = Time.from_msg(odom.header.stamp)
                front_time = Time.from_msg(self.__history_list[0].header.stamp)
                while (current_time - front_time > self.__odom_history_duration):
                    o = self.__history_list[0]
                    self.__odom_cumulate.twist.twist.linear.x -= o.twist.twist.linear.x
                    self.__odom_cumulate.twist.twist.linear.y -= o.twist.twist.linear.y
                    self.__odom_cumulate.twist.twist.linear.z -= o.twist.twist.linear.z
                    self.__odom_cumulate.twist.twist.angular.x -= o.twist.twist.angular.x
                    self.__odom_cumulate.twist.twist.angular.y -= o.twist.twist.angular.y
                    self.__odom_cumulate.twist.twist.angular.z -= o.twist.twist.angular.z
                    del self.__history_list[0]

                    if(not self.__history_list):
                        break

                    front_time = Time.from_msg(self.__history_list[0].header.stamp)
            self.__history_list.append(odom)
            self.__update_state(odom)

    def __update_state(self, odom: Odometry):
        self.__odom_cumulate.twist.twist.linear.x += odom.twist.twist.linear.x
        self.__odom_cumulate.twist.twist.linear.y += odom.twist.twist.linear.y
        self.__odom_cumulate.twist.twist.linear.z += odom.twist.twist.linear.z
        self.__odom_cumulate.twist.twist.angular.x += odom.twist.twist.angular.x
        self.__odom_cumulate.twist.twist.angular.y += odom.twist.twist.angular.y
        self.__odom_cumulate.twist.twist.angular.z += odom.twist.twist.angular.z

        # calculate smoothed twist
        self.__twist_smoothed.header = odom.header
        self.__twist_smoothed.twist.linear.x =\
            self.__odom_cumulate.twist.twist.linear.x / len(self.__history_list)
        self.__twist_smoothed.twist.linear.y =\
            self.__odom_cumulate.twist.twist.linear.y / len(self.__history_list)
        self.__twist_smoothed.twist.linear.z =\
            self.__odom_cumulate.twist.twist.linear.z / len(self.__history_list)
        self.__twist_smoothed.twist.angular.x =\
            self.__odom_cumulate.twist.twist.angular.x / len(self.__history_list)
        self.__twist_smoothed.twist.angular.y =\
            self.__odom_cumulate.twist.twist.angular.y / len(self.__history_list)
        self.__twist_smoothed.twist.angular.z =\
            self.__odom_cumulate.twist.twist.angular.z / len(self.__history_list)
        
        current_ts = datetime.now()
        if(int((current_ts - self.__last_kb_update).total_seconds() * 1000) >= self.__kb_update_rate_ms):
            filter = eval(self.__kb_filter)
            update = {self.__slot: {'ts': Clock().now().nanoseconds, 'data': message_converter.\
                convert_ros_message_to_dictionary(self.__twist_smoothed)}}
            self._kb_server.update(filter, update)
            self.__last_kb_update = current_ts
