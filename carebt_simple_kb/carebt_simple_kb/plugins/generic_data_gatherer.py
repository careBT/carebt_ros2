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

from carebt_simple_kb.plugin_base import import_class
from carebt_simple_kb.plugin_base import PluginBase
import functools
from rclpy.clock import Clock
from rclpy_message_converter import message_converter

KB_DATA_GATHERER_LIST_PARAM = 'DataGatherer.data_gatherer'


class GenericDataGatherer(PluginBase):

    def init_callback(self, plugin_name: str):
        self._kb_server.declare_parameter(KB_DATA_GATHERER_LIST_PARAM, [
                                          ''])  # TODO: empty string list

        # create data_gatherer
        for dg in filter(lambda name: len(name) >= 3,
                         self._kb_server.get_parameter(KB_DATA_GATHERER_LIST_PARAM)
                         .get_parameter_value().string_array_value):
            self._kb_server.get_logger().info(f'GenericDataGatherer - init data_gatherer: {dg}')
            self._kb_server.declare_parameter(f'{plugin_name}.{dg}.topic', '')
            self._kb_server.declare_parameter(f'{plugin_name}.{dg}.type', '')
            self._kb_server.declare_parameter(f'{plugin_name}.{dg}.slot', '')
            self._kb_server.declare_parameter(f'{plugin_name}.{dg}.max_items', 1)
            self._kb_server.declare_parameter(f'{plugin_name}.{dg}.kb_filter', '')
            topic = self._kb_server.get_parameter(
                f'{plugin_name}.{dg}.topic').get_parameter_value().string_value
            msg_type = self._kb_server.get_parameter(
                f'{plugin_name}.{dg}.type').get_parameter_value().string_value
            slot = self._kb_server.get_parameter(
                f'{plugin_name}.{dg}.slot').get_parameter_value().string_value
            max_items = self._kb_server.get_parameter(
                f'{plugin_name}.{dg}.max_items').get_parameter_value().integer_value
            kb_filter = self._kb_server.get_parameter(
                f'{plugin_name}.{dg}.kb_filter').get_parameter_value().string_value

            self._kb_server.get_logger().info(f'GenericDataGatherer - {dg} topic= {topic}')
            self._kb_server.get_logger().info(f'GenericDataGatherer - {dg} msg_type= {msg_type}')
            self._kb_server.get_logger().info(f'GenericDataGatherer - {dg} slot= {slot}')
            self._kb_server.get_logger().info(f'GenericDataGatherer - {dg} max_items= {max_items}')
            self._kb_server.get_logger().info(f'GenericDataGatherer - {dg} kb_filter= {kb_filter}')

            msg_class = import_class(msg_type)
            self._kb_server.create_subscription(msg_class, topic, functools.partial(
                self.data_gatherer_callback, slot, kb_filter, max_items), 10)

    def data_gatherer_callback(self, slot, kb_filter, max_items, msg):
        self._kb_server.get_logger().debug(
            f'GenericDataGatherer - Incoming topic= {msg}; slot= {slot}, max_items= {max_items}')
        msg_dict = message_converter.convert_ros_message_to_dictionary(msg)
        filter = eval(kb_filter)
        if(max_items == 1):    
            update = {slot: {'ts': Clock().now().nanoseconds, 'data': msg_dict}}
        else:
            result = self._kb_server.read(filter)
            items = []
            if(len(result) >= 1):
                items = result[0][slot]
            items.append({'ts': Clock().now().nanoseconds, 'data': msg_dict})
            if(len(items) > max_items):
                del items[0]
            update = {slot: items}
        # update kb
        self._kb_server.update(filter, update)
