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

import sys
import threading
from time import monotonic
import traceback

from carebt import BehaviorTreeRunner
from carebt import LogLevel
from carebt import NodeStatus
from carebt import TreeNode
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.executors import ShutdownException
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

from carebt_ros2.rosClientManager import RosClientManager
from carebt_ros2.rosLogger import RosLogger

# longest time the loop waits for ROS callbacks, limits the reaction time on a shutdown
_MAX_SPIN_S = 0.1
# minimum time between two warnings about slow callbacks or ticks
_SLOW_WARN_INTERVAL_S = 1.0
# time between two repeated reports of a blocked loop
_BLOCKED_REPORT_INTERVAL_S = 5.0


class RosCarebtRunner(Node):
    """Runs a careBT behavior tree and the callbacks of its ROS 2 node in one thread.

    The thread alternately spins the ROS node, until the next tick is due, and ticks the
    behavior tree. Thus, ROS callbacks (subscriptions, services, actions, futures) and
    careBT callbacks are never executed in parallel, and the state of the nodes can
    safely be changed in the callbacks of ROS.

    Consequently, no callback and no `on_tick` must block or take a long time. Do not use
    `wait_for_*`, `client.call`, `future.result` without a done callback, `sleep` or any
    other blocking call. Use `call_async` and `async` callbacks instead.

    A watchdog thread, which only reads, logs the stack of the loop thread in case the loop
    does not make progress for `watchdog_timeout_s` seconds.

    Parameters
    ----------
    node_name: str
        The name of the ROS node
    watchdog_timeout_s: float, optional
        Time without progress after which the watchdog reports. None disables it.

    """

    def __init__(self, node_name: str = 'carebt_runner', watchdog_timeout_s: float = 1.0):
        rclpy.init(args=None)
        Node.__init__(self, node_name)

        self.client_manager = RosClientManager(self)

        self.__executor = SingleThreadedExecutor()
        self.__executor.add_node(self)
        self.__bt_runner = BehaviorTreeRunner()
        self.__bt_runner.node = self
        self.__bt_runner.set_logger(RosLogger(self.get_logger()))
        self.__bt_runner.get_logger().set_log_level(LogLevel.INFO)

        self.__watchdog_timeout_s = watchdog_timeout_s
        self.__heartbeat = monotonic()
        self.__phase = 'idle'
        self.__last_slow_warning = 0.0

    def destroy_node(self) -> bool:
        self.client_manager.shutdown()
        self.__executor.remove_node(self)
        return super().destroy_node()

    def run(self, node: TreeNode, params: str = None) -> None:
        """
        Execute the provided node, respectively the provided behavior tree.

        Blocks until the behavior tree has completed or ROS is shut down. In the meantime
        the calling thread spins the ROS node and ticks the behavior tree with the tick rate
        of the `BehaviorTreeRunner` (default 50 ms). `run` must always be called from the
        same thread.

        Parameters
        ----------
        node: TreeNode
            The node which should be executed
        params: str, optional
            The parameters for the node which should be executed

        """
        period = self.__bt_runner.get_tick_rate_ms() / 1000
        self.__bt_runner.initialize(node, params)

        stop_watchdog = threading.Event()
        self.__heartbeat = monotonic()
        if self.__watchdog_timeout_s:
            threading.Thread(target=self.__watchdog,
                             args=(threading.get_ident(), stop_watchdog),
                             daemon=True).start()

        next_tick = monotonic()
        try:
            while rclpy.ok():
                # spin until the next tick is due
                while (remaining := next_tick - monotonic()) > 0:
                    self.__spin_once(min(remaining, _MAX_SPIN_S), period)

                status = self.__tick(period)
                if status != NodeStatus.IDLE and status != NodeStatus.RUNNING:
                    break

                # fixed rate, but do not try to catch up after an overrun
                next_tick += period
                if next_tick < monotonic():
                    next_tick = monotonic()
        except (ExternalShutdownException, ShutdownException, KeyboardInterrupt):
            # shutdown requested: the ROS context is already invalid, so there is nothing to log
            pass
        finally:
            stop_watchdog.set()
            self.__phase = 'idle'

    def get_bt_runner(self) -> BehaviorTreeRunner:
        return self.__bt_runner

    # PRIVATE

    def __spin_once(self, timeout_s: float, period: float) -> None:
        self.__phase = 'ROS callback'
        start = monotonic()
        try:
            self.__executor.spin_once(timeout_sec=timeout_s)
        except (ExternalShutdownException, ShutdownException, KeyboardInterrupt):
            raise
        except Exception:
            # an error in a callback must not stop the behavior tree
            self.get_logger().error(f'exception in ROS callback:\n{traceback.format_exc()}')
        self.__heartbeat = monotonic()
        # spin_once returns latest after the timeout, if there was nothing to do
        self.__warn_if_slow('ROS callback', self.__heartbeat - start - timeout_s, period)

    def __tick(self, period: float) -> NodeStatus:
        self.__phase = 'tick'
        start = monotonic()
        try:
            status = self.__bt_runner.tick()
        except Exception:
            self.get_logger().error(f'exception in tick:\n{traceback.format_exc()}')
            raise
        self.__heartbeat = monotonic()
        self.__warn_if_slow('tick', self.__heartbeat - start, period)
        return status

    def __warn_if_slow(self, what: str, duration_s: float, period: float) -> None:
        now = monotonic()
        if duration_s > period and now - self.__last_slow_warning > _SLOW_WARN_INTERVAL_S:
            self.__last_slow_warning = now
            self.get_logger().warn(f'{what} took {duration_s * 1000:.0f} ms, '
                                   f'the tick period is {period * 1000:.0f} ms. '
                                   'Callbacks and ticks must not block or take long.')

    def __watchdog(self, loop_thread_id: int, stop: threading.Event) -> None:
        """Read-only: reports a loop which does not make progress, never touches the tree."""
        last_report = None
        while not stop.wait(self.__watchdog_timeout_s / 4):
            blocked_s = monotonic() - self.__heartbeat
            if blocked_s > self.__watchdog_timeout_s:
                now = monotonic()
                if last_report is None or now - last_report >= _BLOCKED_REPORT_INTERVAL_S:
                    last_report = now
                    frame = sys._current_frames().get(loop_thread_id)
                    stack = ''.join(traceback.format_stack(frame)) if frame else 'n/a'
                    self.get_logger().warn(f'the loop is blocked for {blocked_s:.1f} s in '
                                           f'{self.__phase}. Stack of the loop thread:\n{stack}')
            elif last_report is not None:
                last_report = None
                self.get_logger().info('the loop is running again')
