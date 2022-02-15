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
from carebt_ros2.rosSubscriberActionNode import RosSubscriberActionNode
from lifecycle_msgs.srv import ChangeState
from std_msgs.msg import Empty
import threading

########################################################################


class NoopAction(ActionNode):
    """A noop action node.

    Noop node  which can, for example, be used to register a contingency
    handler on to trigger a callback function.

    """

    def __init__(self, bt_runner):
        super().__init__(bt_runner)

    def on_tick(self) -> None:
        self.set_status(NodeStatus.SUCCESS)

########################################################################


class WaitAction(ActionNode):
    """Wait until time has elapsed.

    Waits until the provided time in milliseconds has elapsed.

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

########################################################################


class WaitForUserInput(RosSubscriberActionNode):
    """Wait for an user input.

    Waits until an user input has been received.

    """

    def __init__(self, bt_runner):
        super().__init__(bt_runner, Empty, 'wait_at_waypoint', '')
        self.get_logger().info('{} - wait for user input...'.format(self.__class__.__name__))

    def topic_callback(self, msg) -> None:
        self.get_logger().info('{} - user input received.'.format(self.__class__.__name__))
        self.set_status(NodeStatus.SUCCESS)

########################################################################


class LifecycleClient(ActionNode):
    """Changes the state of a Lifecycle Node.

    Input Parameters
    ----------------
    ?service : str
        The nodes lifecycle service name
    ?id : int
        The status (id) to change into

    """

    def __init__(self, bt_runner):
        super().__init__(bt_runner, '?service ?id')

    def on_init(self) -> None:
        self.get_logger().info('{} - put {} into state {}'
                               .format(self.__class__.__name__, self._service, self._id))
        threading.Thread(target=self.__worker, daemon=True).start()
        self.set_status(NodeStatus.SUSPENDED)

    def __worker(self):
        self.__client = self.create_client(ChangeState, self._service)
        if(self.__client.wait_for_service(timeout_sec=1.0)):
            req = ChangeState.Request()
            req.transition.id = self._id
            res: ChangeState.Response = self.__client.call(req)
            if(res.success):
                self.set_status(NodeStatus.SUCCESS)
            else:
                self.set_status(NodeStatus.FAILURE)
                self.set_contingency_message('CHANGE_STATE_FAILED')
        else:
            self.get_logger().warn('service not available')
            self.set_status(NodeStatus.FAILURE)
            self.set_contingency_message('SERVICE_NOT_AVAILABLE')

    def __del__(self) -> None:
        self.__client.destroy()
