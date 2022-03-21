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

from carebt.actionNode import ActionNode
from carebt.nodeStatus import NodeStatus
from carebt_ros2.rosSubscriberActionNode import RosSubscriberActionNode
from lifecycle_msgs.srv import ChangeState, GetState
from rcl_interfaces.msg import Parameter
from rcl_interfaces.srv import SetParameters
from ros2param.api import get_parameter_value
from std_msgs.msg import Empty
from threading import Timer, Thread
from time import sleep

########################################################################


class NoopAction(ActionNode):
    """A noop action node.

    Noop node which can, for example, be used to register a contingency
    handler on to trigger a callback function. The `NoopAction` returns the
    status and contingency message which are provided as input. By default it
    returns `SUCCESS` with an empty contingency message.

    Input Parameters
    ----------------
    ?status : NodeStatus
        NodeStatus to return

    ?contingency_message : str
        Contingency message to return

    """

    def __init__(self, bt_runner):
        super().__init__(bt_runner, "?status ?contingency_message")
        self._status = NodeStatus.SUCCESS
        self._contingency_message = ""

    def on_init(self) -> None:
        self.set_status(self._status)
        self.set_contingency_message(self._contingency_message)

########################################################################


class WaitAction(ActionNode):
    """Wait until time has elapsed.

    Waits until the provided time in milliseconds has elapsed.

    Input Parameters
    ----------------
    ?time : int (ms)
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
    ?node : str
        The nodes name
    ?id : int
        The status (id) to change into

    """

    def __init__(self, bt_runner):
        super().__init__(bt_runner, '?node ?id')
        self.__bt_runner = bt_runner
        self.set_timeout(5000)

    def on_init(self) -> None:
        self.get_logger().info('{} - put {} into state {}'
                               .format(self.__class__.__name__, self._node, self._id))
        self.__thread_running = True
        self.__expected_goal_state = [0, 2, 1, 3, 2, 4, 4, 4][self._id]
        Thread(target=self.__worker, daemon=True).start()
        self.set_status(NodeStatus.SUSPENDED)

    def __worker(self):
        self.__change_state_client =\
            self.__bt_runner.node.create_client(ChangeState, f'/{self._node}/change_state')
        self.__get_state_client =\
            self.__bt_runner.node.create_client(GetState, f'/{self._node}/get_state')
        if(self.__change_state_client.wait_for_service(timeout_sec=1.0)
           and self.__get_state_client.wait_for_service(timeout_sec=1.0)):
            req = ChangeState.Request()
            req.transition.id = self._id
            res: ChangeState.Response = self.__change_state_client.call(req)
            if not res.success:
                self.set_status(NodeStatus.FAILURE)
                self.set_contingency_message('CHANGE_STATE_FAILED')
        else:
            self.get_logger().warn('service not available')
            self.set_status(NodeStatus.FAILURE)
            self.set_contingency_message('SERVICE_NOT_AVAILABLE')

        while True:
            if(not self.__thread_running):
                break
            req = GetState.Request()
            res: GetState.Response = self.__get_state_client.call(req)
            if res.current_state.id == self.__expected_goal_state:
                self.set_status(NodeStatus.SUCCESS)
                break
            sleep(0.1)


    def on_timeout(self) -> None:
        self.__thread_running = False
        self.set_status(NodeStatus.FAILURE)
        self.set_contingency_message('TIMEOUT')

    def __del__(self) -> None:
        self.__change_state_client.destroy()
        self.__get_state_client.destroy()

########################################################################


class SetParameterClient(ActionNode):
    """Set the parameter (name/value) of the node.

    Input Parameters
    ----------------
    ?node : str
        The nodes name
    ?param_name : str
        The parameters name
    ?param_value
        The parameters value

    """

    def __init__(self, bt_runner):
        super().__init__(bt_runner, '?node ?param_name ?param_value')
        self.__bt_runner = bt_runner

    def on_init(self) -> None:
        self.get_logger().info('{} - {} set param {} to {}'
                               .format(self.__class__.__name__,
                                       self._node,
                                       self._param_name,
                                       self._param_value))
        Thread(target=self.__worker, daemon=True).start()
        self.set_status(NodeStatus.SUSPENDED)

    def __worker(self):
        self.__client = self.__bt_runner.node.create_client(SetParameters,
                                                            f'/{self._node}/set_parameters')
        if(self.__client.wait_for_service(timeout_sec=1.0)):
            param = Parameter()
            param.name = self._param_name
            param.value = get_parameter_value(string_value=self._param_value)

            req = SetParameters.Request()
            req.parameters = [param]

            resp = self.__client.call(req)
            if resp.results[0].successful:
                self.set_status(NodeStatus.SUCCESS)
            else:
                self.set_status(NodeStatus.FAILURE)
                self.set_contingency_message('PARAM_NOT_SET')
        else:
            self.set_status(NodeStatus.FAILURE)
            self.set_contingency_message('SERVICE_NOT_AVAILABLE')

########################################################################


class ServiceClient(ActionNode):
    """Call a ROS2 service.

    Input Parameters
    ----------------
    ?service : str
        The ROS2 service name
    ?request
        The service request msg

    """

    def __init__(self, bt_runner):
        super().__init__(bt_runner, '?service ?type ?request => ?response')
        self.__bt_runner = bt_runner

    def on_init(self) -> None:
        self.get_logger().info('{} - call service {} with {} - {}'
                               .format(self.__class__.__name__,
                                       self._service,
                                       self._type,
                                       self._request))
        Thread(target=self.__worker, daemon=True).start()
        self.set_status(NodeStatus.SUSPENDED)

    def __worker(self):
        self.__client = self.__bt_runner.node.create_client(self._type, self._service)
        if(self.__client.wait_for_service(timeout_sec=1.0)):
            req = self._request
            self._response = self.__client.call(req)
            self.set_status(NodeStatus.SUCCESS)
        else:
            self.set_status(NodeStatus.FAILURE)
            self.set_contingency_message('SERVICE_NOT_AVAILABLE')
