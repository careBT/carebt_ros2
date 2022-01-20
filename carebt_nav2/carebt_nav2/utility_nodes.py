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

from threading import Timer

from carebt.actionNode import ActionNode
from carebt.nodeStatus import NodeStatus

########################################################################


class WaitAction(ActionNode):
    """Wait until time has elapsed.

    Waits until the provided time in milliseconds has elapsd.

    Input Parameters
    ----------------
    ?ime : int (ms)
        Milliseconds to wait

    """

    def __init__(self, bt_runner):
        super().__init__(bt_runner, '?time')

    def on_tick(self) -> None:
        self.get_logger().info('{} - wait for {} milliseconds...'
                               .format(self.__class__.__name__, self._time))
        self.set_status(NodeStatus.SUSPENDED)
        self.__done_timer = Timer(self._time / 1000, self.done_callback)
        self.__done_timer.start()

    def done_callback(self) -> None:
        self.get_logger().info('{} - waiting done.'.format(self.__class__.__name__))
        self.set_status(NodeStatus.SUCCESS)

    def on_abort(self) -> None:
        self.get_logger().info('{} - waiting aborted.'.format(self.__class__.__name__))
        self.__done_timer.cancel()

    def on_delete(self) -> None:
        # set the timer to None to make sure that all references (bound method)
        # are released and the object gets destroyed by gc
        self.__done_timer = None
