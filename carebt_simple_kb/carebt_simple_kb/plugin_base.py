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

from rclpy.node import Node
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from carebt_simple_kb.carebt_simple_kb import KbServer  # pragma: no cover

def import_class(name: str) -> str:
    components = name.split('.')
    mod = __import__(".".join(components[0:-1]), fromlist=components[-1])
    clazz = getattr(mod, components[-1])
    return clazz


class PluginBase():

    def __init__(self, kb_server: 'KbServer', plugin_name: str):
        self._kb_server = kb_server
        self.init_callback(plugin_name)

    # PUBLIC

    def init_callback(self, plugin_name: str):
        pass