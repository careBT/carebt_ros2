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
from carebt_ros2.rosServiceCallActionNode import RosServiceCallActionNode
from carebt_ros2.rosSubscriberActionNode import RosSubscriberActionNode
from lifecycle_msgs.srv import ChangeState, GetState
from rcl_interfaces.srv import GetParameters, SetParameters
from std_msgs.msg import Empty
from rclpy.parameter import Parameter, parameter_value_to_python

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
        self.create_timer(self._time, self.done_callback)

    def done_callback(self) -> None:
        self.get_logger().info('{} - waiting done.'.format(self.__class__.__name__))
        self.set_status(NodeStatus.SUCCESS)

    def on_abort(self) -> None:
        self.get_logger().info('{} - waiting aborted.'.format(self.__class__.__name__))

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


class LifecycleClient(RosServiceCallActionNode):
    """Changes the state of a Lifecycle Node.
    Transition ids:
    0: create
    1: configure
    2: cleanup
    3: activate
    4: deactivate
    5: unconfigured_shutdown
    6: inactive_shutdown
    7: active_shutdown
    8: destroy
    
    State ids:
    0: unknown
    1: unconfigured
    2: inactive
    3: active
    4: finalized

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
        self.set_timeout(25000)

    def on_init(self) -> None:
        self.get_logger().info('{} - put {} into state {}'
                               .format(self.__class__.__name__, self._node, self._id))
        self.set_status(NodeStatus.SUSPENDED)
        # self.__expected_goal_state = [0, 2, 1, 3, 2, 4, 4, 4][self._id]
        self.__expected_goal_state = [1, 2, 1, 3, 2, 4, 4, 4, None][self._id]
        client_manager = self.__bt_runner.node.client_manager
        self.__change_state_client = client_manager.get_or_create_service_client(
            ChangeState, f'/{self._node}/change_state')
        self.__get_state_client = client_manager.get_or_create_service_client(
            GetState, f'/{self._node}/get_state')
        req = ChangeState.Request()
        req.transition.id = self._id
        # wait until the service is available, the node timeout limits the waiting time
        self.call_service(self.__change_state_client, req, self.__on_change_state, wait_ms=None)

    def __on_change_state(self, res: ChangeState.Response) -> None:
        if not res.success:
            self.set_status(NodeStatus.FAILURE)
            self.set_contingency_message('CHANGE_STATE_FAILED')
            return
        self.__request_state()

    def __request_state(self) -> None:
        self.call_service(self.__get_state_client, GetState.Request(),
                          self.__on_get_state, wait_ms=None)

    def __on_get_state(self, res: GetState.Response) -> None:
        if res.current_state.id == self.__expected_goal_state:
            self.set_status(NodeStatus.SUCCESS)
        else:
            self.create_timer(100, self.__request_state)

    def on_timeout(self) -> None:
        self.set_status(NodeStatus.FAILURE)
        self.set_contingency_message('TIMEOUT')

########################################################################


class SetParameterClient(RosServiceCallActionNode):
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
        self.set_status(NodeStatus.SUSPENDED)
        client = self.__bt_runner.node.client_manager.get_or_create_service_client(
            SetParameters, f'/{self._node}/set_parameters')
        param = Parameter(name=self._param_name, value=self._param_value)
        req = SetParameters.Request()
        req.parameters = [param.to_parameter_msg()]
        self.get_logger().info(f'Calling set_parameters service with {req.parameters}...')
        self.call_service(client, req, self.__on_response)

    def __on_response(self, resp: SetParameters.Response) -> None:
        if resp.results[0].successful:
            self.set_status(NodeStatus.SUCCESS)
        else:
            self.set_status(NodeStatus.FAILURE)
            self.set_contingency_message('PARAM_NOT_SET')

########################################################################

class SetParameterListClient(RosServiceCallActionNode):
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
        super().__init__(bt_runner, '?node ?param_list')
        self.__bt_runner = bt_runner

    def on_init(self) -> None:
        self.get_logger().info('{} - {} setting params {}'
                               .format(self.__class__.__name__,
                                       self._node,
                                       [(param.name,param.value) for param in self._param_list]))
        self.set_status(NodeStatus.SUSPENDED)
        client = self.__bt_runner.node.client_manager.get_or_create_service_client(
            SetParameters, f'/{self._node}/set_parameters')
        parameter_msgs = []
        for param in self._param_list:
            if isinstance(param, Parameter): parameter_msgs.append(param.to_parameter_msg())
            else: self.get_logger().warn(f"Parameter {param} is not of type rclpy.Parameter and will be ignored.")

        req = SetParameters.Request()
        req.parameters = parameter_msgs
        self.call_service(client, req, self.__on_response)

    def __on_response(self, resp: SetParameters.Response) -> None:
        if all(result.successful for result in resp.results):
            self.set_status(NodeStatus.SUCCESS)
        else:
            self.set_status(NodeStatus.FAILURE)
            self.set_contingency_message('PARAM_NOT_SET')


class GetParameterClient(RosServiceCallActionNode):
    """Get the parameter (name/value) of the node.

    Input Parameters
    ----------------
    ?node : str
        The nodes name
    ?param_name : str
        The parameters name

    """

    def __init__(self, bt_runner):
        super().__init__(bt_runner, '?node ?param_name => ?param_value')
        self.__bt_runner = bt_runner

    def on_init(self) -> None:
        self.get_logger().info('{} - {} get param {}'
                               .format(self.__class__.__name__,
                                       self._node,
                                       self._param_name))
        self.set_status(NodeStatus.SUSPENDED)
        client = self.__bt_runner.node.client_manager.get_or_create_service_client(
            GetParameters, f'/{self._node}/get_parameters')
        req = GetParameters.Request()
        req.names = [self._param_name]
        self.call_service(client, req, self.__on_response)

    def __on_response(self, resp: GetParameters.Response) -> None:
        if len(resp.values) > 0:
            self.set_status(NodeStatus.SUCCESS)
            self._param_value = parameter_value_to_python(resp.values[0])
            self.get_logger().info('{} - {} got param {} with value {}'
                                   .format(self.__class__.__name__,
                                           self._node,
                                           self._param_name,
                                           self._param_value))
        else:
            self.set_status(NodeStatus.FAILURE)
            self.set_contingency_message('PARAM_NOT_FOUND')

