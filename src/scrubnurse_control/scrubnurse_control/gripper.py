"""
Gripper backends sharing one interface: activate() and blocking move(position, speed, force).

Positions, speeds and forces use the Robotiq 0..255 scale (position 0 = open).
"""

from collections import OrderedDict
from enum import Enum
import socket
import threading
import time


class RobotiqSocketGripper:
    """
    Robotiq Hand-E / 2F gripper through the Robotiq URCap socket on the UR controller.

    Same protocol as the ROS 1 controller (SET/GET of ACT, GTO, POS, ... on TCP
    port 63352), with timeouts so a silent gripper cannot hang the controller.
    """

    ACT, GTO, ATR, FOR, SPE, POS = 'ACT', 'GTO', 'ATR', 'FOR', 'SPE', 'POS'
    STA, PRE, OBJ, FLT = 'STA', 'PRE', 'OBJ', 'FLT'
    ENCODING = 'UTF-8'

    class Status(Enum):
        RESET = 0
        ACTIVATING = 1
        ACTIVE = 3

    class ObjectStatus(Enum):
        MOVING = 0
        STOPPED_OUTER_OBJECT = 1
        STOPPED_INNER_OBJECT = 2
        AT_DEST = 3

    def __init__(self, host, port=63352, timeout=10.0, log=print):
        self._host, self._port = host, port
        self._timeout = timeout
        self._log = log
        self._socket = None
        self._lock = threading.Lock()
        self._min_position, self._max_position = 0, 255

    def connect(self):
        self._socket = socket.create_connection((self._host, self._port), timeout=2.0)

    def disconnect(self):
        if self._socket is not None:
            self._socket.close()
            self._socket = None

    def activate(self, auto_calibrate=True):
        """Connect, activate if needed and (like ROS 1) calibrate the open/closed range."""
        if self._socket is None:
            self.connect()
        if self._get(self.STA) == self.Status.ACTIVE.value:
            return
        self._reset()
        self._set(self.ACT, 1)
        time.sleep(1.0)
        self._wait_until(lambda: self._get(self.ACT) == 1 and self._get(self.STA) == 3,
                         'activation')
        if auto_calibrate:
            self._auto_calibrate()

    def move(self, position, speed, force):
        """Move and wait until the gripper stops (at target or on an object)."""
        position = max(self._min_position, min(int(position), self._max_position))
        speed = max(0, min(int(speed), 255))
        force = max(0, min(int(force), 255))
        if not self._set_many(OrderedDict(
                [(self.POS, position), (self.SPE, speed), (self.FOR, force), (self.GTO, 1)])):
            return False
        self._wait_until(lambda: self._get(self.PRE) == position, 'position request')
        self._wait_until(lambda: self._get(self.OBJ) != self.ObjectStatus.MOVING.value, 'motion')
        return True

    def _reset(self):
        self._set(self.ACT, 0)
        self._set(self.ATR, 0)

        def is_reset():
            if self._get(self.ACT) == 0 and self._get(self.STA) == 0:
                return True
            self._set(self.ACT, 0)
            self._set(self.ATR, 0)
            return False

        self._wait_until(is_reset, 'reset')
        time.sleep(0.5)

    def _auto_calibrate(self):
        for target in (self._min_position, self._max_position, self._min_position):
            self.move(target, 64, 1)
            status = self._get(self.OBJ)
            if status != self.ObjectStatus.AT_DEST.value:
                raise RuntimeError(f'gripper calibration blocked by an object (OBJ={status})')
            if target == self._max_position:
                self._max_position = self._get(self.POS)
            else:
                self._min_position = self._get(self.POS)
        self._log(f'Gripper calibrated to [{self._min_position}, {self._max_position}]')

    def _wait_until(self, condition, what):
        deadline = time.monotonic() + self._timeout
        while not condition():
            if time.monotonic() > deadline:
                raise TimeoutError(f'gripper {what} timed out after {self._timeout:.0f} s')
            time.sleep(0.01)

    def _set_many(self, values):
        cmd = 'SET' + ''.join(f' {k} {v}' for k, v in values.items()) + '\n'
        with self._lock:
            self._socket.sendall(cmd.encode(self.ENCODING))
            return self._socket.recv(1024) == b'ack'

    def _set(self, variable, value):
        return self._set_many(OrderedDict([(variable, value)]))

    def _get(self, variable):
        with self._lock:
            self._socket.sendall(f'GET {variable}\n'.encode(self.ENCODING))
            data = self._socket.recv(1024)
        reply = data.decode(self.ENCODING, errors='replace').split()
        if len(reply) != 2 or reply[0] != variable or not reply[1].isdigit():
            # An OSError, so the controller pauses instead of crashing.
            raise ConnectionError(f'unexpected gripper reply {data!r} to GET {variable}')
        return int(reply[1])


class ActionGripper:
    """Gripper behind a control_msgs/GripperCommand action, e.g. ros2_control in simulation."""

    def __init__(self, node, action_name, open_position, closed_position, max_effort,
                 callback_group=None, timeout=10.0):
        from control_msgs.action import GripperCommand
        from rclpy.action import ActionClient

        self._goal_type = GripperCommand.Goal
        self._action_name = action_name
        self._client = ActionClient(node, GripperCommand, action_name,
                                    callback_group=callback_group)
        self._open, self._closed = open_position, closed_position
        self._max_effort = max_effort
        self._timeout = timeout
        self._logger = node.get_logger()

    def activate(self):
        if not self._client.wait_for_server(timeout_sec=self._timeout):
            raise TimeoutError(f'gripper action server {self._action_name} not available')

    def move(self, position, speed, force):
        goal = self._goal_type()
        goal.command.position = self._open + (self._closed - self._open) * position / 255.0
        goal.command.max_effort = self._max_effort
        handle = self._wait(self._client.send_goal_async(goal))
        if handle is None or not handle.accepted:
            self._logger.error('gripper goal rejected or timed out')
            return False
        response = self._wait(handle.get_result_async())
        if response is None:
            return False
        result = response.result
        # Stalling on a tool is a successful grasp.
        return result.reached_goal or result.stalled

    def _wait(self, future):
        deadline = time.monotonic() + self._timeout
        while not future.done():
            if time.monotonic() > deadline:
                return None
            time.sleep(0.01)
        return future.result()


class MockGripper:
    """Gripper stand-in for motion-only testing (MoveIt with mock hardware)."""

    def __init__(self, log=print, duration=0.5):
        self._log = log
        self._duration = duration

    def activate(self):
        self._log('Mock gripper active')

    def move(self, position, speed, force):
        self._log(f'Mock gripper -> {position}')
        time.sleep(self._duration)
        return True
