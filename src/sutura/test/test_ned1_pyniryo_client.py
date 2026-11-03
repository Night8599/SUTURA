"""
Ned1ArmDriver with the REAL pyniryo 1.0.5 client against a fake Niryo TCP server on localhost.

This checks what FakeNiryoRobot cannot: the driver's calls match the real client's signatures,
and the real exception classes (HostNotReachableException, NiryoRobotException, ...) are
classified correctly. No robot is involved.

Wire protocol of pyniryo 1.0.5 (pyniryo/api/communication_functions.py): a 2-byte native-endian
length ('@H') followed by a JSON request {"command": NAME, "param_list": [...]}. The reply uses
the same framing and holds the repr of {"status": "OK" | "KO", "list_ret_param": [...]}.

The whole module is skipped when pyniryo is not installed (it is not in install_requires, see
docs/drivers/ned1.md).
"""

import json
import socket
import struct
import threading
import time

import pytest

try:
    import pyniryo
except ImportError:
    pytest.skip(
        "pyniryo is not installed: skipping the real-client tests "
        "(pip install --no-deps \"pyniryo==1.0.5\" and pip install numpy opencv-python, see docs/drivers/ned1.md)",
        allow_module_level=True,
    )

from sutura.drivers.ned1_driver import Ned1ArmDriver  # noqa: E402

HOME = [0.0, 0.3, -1.3, 0.0, 0.0, 0.0]


class FakeNiryoServer:
    """Single-client fake of the Niryo TCP server (port is ephemeral)."""

    def __init__(self, calibration_needed=False):
        self.joints = list(HOME)
        self.calibration_needed = calibration_needed
        self.requests = []      # (command, param_list) in arrival order
        self.ko_on = None       # command answered with status KO
        self.drop_on = None     # command after which the connection is closed without a reply
        self.move_delay = 0.0   # seconds a MOVE_JOINTS takes to be answered (motions are blocking)
        self._listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._listener.bind(('127.0.0.1', 0))
        self._listener.listen(1)
        self._listener.settimeout(5.0)
        self.port = self._listener.getsockname()[1]
        self._conn = None
        self._thread = threading.Thread(target=self._serve, daemon=True)
        self._thread.start()

    def commands(self):
        return [name for name, _ in self.requests]

    def close(self):
        for sock in (self._conn, self._listener):
            if sock is not None:
                try:
                    sock.close()
                except OSError:
                    pass
        self._thread.join(timeout=2.0)

    # ------------------------------------------------------------------
    def _serve(self):
        try:
            self._conn, _ = self._listener.accept()
        except OSError:
            return
        self._conn.settimeout(5.0)
        try:
            while True:
                header = self._recv_exact(2)
                if header is None:
                    return
                (size,) = struct.unpack('@H', header)
                body = self._recv_exact(size)
                if body is None:
                    return
                request = json.loads(body.decode())
                command, params = request['command'], request['param_list']
                self.requests.append((command, params))
                reply = self._handle(command, params)
                if command == self.drop_on:
                    # Close the socket for real: the client sees the link die while waiting for
                    # the reply. (pyniryo 1.0.5 sets no socket timeout after connecting, so a
                    # peer that just goes silent would block it forever.)
                    self._conn.shutdown(socket.SHUT_RDWR)
                    self._conn.close()
                    return
                payload = repr(reply).encode()
                self._conn.sendall(struct.pack('@H', len(payload)) + payload)
        except OSError:
            return

    def _recv_exact(self, count):
        data = b''
        while len(data) < count:
            chunk = self._conn.recv(count - len(data))
            if not chunk:
                return None
            data += chunk
        return data

    def _handle(self, command, params):
        if command == self.ko_on:
            return {'status': 'KO', 'message': f'{command} refused by the fake robot'}
        ok = {'status': 'OK', 'list_ret_param': []}
        if command == 'GET_HARDWARE_STATUS':
            ok['list_ret_param'] = [
                40.0, 1, 'False', '', str(self.calibration_needed), 'False',
                ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6'],
                ['Niryo Stepper'] * 3 + ['DXL XL-430'] * 2 + ['DXL XL-320'],
                [30.0] * 6, [12.0] * 6, [0] * 6,
            ]
        elif command == 'CALIBRATE':
            self.calibration_needed = False
        elif command == 'GET_JOINTS':
            ok['list_ret_param'] = list(self.joints)
        elif command == 'MOVE_JOINTS':
            time.sleep(self.move_delay)
            self.joints = [float(v) for v in params]
        elif command in ('SET_LEARNING_MODE', 'SET_ARM_MAX_VELOCITY'):
            pass
        else:
            return {'status': 'KO', 'message': f'unknown command {command}'}
        return ok


@pytest.fixture
def server():
    srv = FakeNiryoServer()
    yield srv
    srv.close()


def make_driver(server, **kwargs):
    def factory():
        robot = pyniryo.NiryoRobot()
        robot._NiryoRobot__port = server.port  # pyniryo 1.0.5 hard-codes TCP port 40001
        return robot

    return Ned1ArmDriver(ip_address='127.0.0.1', client_factory=factory, **kwargs)


def test_pyniryo_version_is_1_0_5():
    from importlib.metadata import version
    assert version('pyniryo') == '1.0.5'


def test_connect_calibrate_and_configure(server):
    server.calibration_needed = True
    driver = make_driver(server, velocity_percent=15)
    try:
        assert driver.connect(), driver.last_error
        assert ('CALIBRATE', ['AUTO']) in server.requests
        assert ('SET_LEARNING_MODE', ['FALSE']) in server.requests
        assert ('SET_ARM_MAX_VELOCITY', [15]) in server.requests
        # Calibration must come before learning mode is turned off.
        assert server.commands().index('CALIBRATE') < server.commands().index('SET_LEARNING_MODE')
        assert driver.get_joint_positions() == HOME
    finally:
        driver.disconnect()


def test_no_calibration_when_not_needed(server):
    driver = make_driver(server)
    try:
        assert driver.connect(), driver.last_error
        assert 'CALIBRATE' not in server.commands()
    finally:
        driver.disconnect()


def test_move_and_read_back(server):
    driver = make_driver(server)
    try:
        assert driver.connect(), driver.last_error
        target = [0.1, 0.3, -1.3, 0.0, 0.0, 0.0]
        assert driver.set_joint_positions(target), driver.last_error
        assert ('MOVE_JOINTS', target) in server.requests
        assert driver.get_joint_positions() == target
    finally:
        driver.disconnect()


def test_state_read_does_not_block_during_a_real_blocking_motion(server):
    server.move_delay = 0.6
    driver = make_driver(server)
    try:
        assert driver.connect(), driver.last_error
        target = [0.1, 0.3, -1.3, 0.0, 0.0, 0.0]
        worker = threading.Thread(target=driver.set_joint_positions, args=(target,))
        worker.start()
        for _ in range(500):
            if driver.is_moving:
                break
            time.sleep(0.002)
        assert driver.is_moving
        started = time.monotonic()
        assert driver.get_joint_positions() == HOME  # cached: the robot cannot be queried mid-motion
        assert time.monotonic() - started < 0.2
        worker.join(timeout=5.0)
        assert not driver.is_moving
        assert driver.get_joint_positions() == target
    finally:
        driver.disconnect()


def test_home_uses_the_vendor_home_pose(server):
    driver = make_driver(server)
    try:
        assert driver.connect(), driver.last_error
        server.joints = [0.2, 0.0, -1.0, 0.0, 0.0, 0.0]
        assert driver.move_to_home(), driver.last_error
        assert server.requests[-2] == ('MOVE_JOINTS', HOME)
        assert driver.get_joint_positions() == HOME
    finally:
        driver.disconnect()


def test_rejected_command_never_reaches_the_robot(server):
    driver = make_driver(server)
    try:
        assert driver.connect(), driver.last_error
        assert not driver.set_joint_positions([3.5, 0.3, -1.3, 0.0, 0.0, 0.0])
        assert 'MOVE_JOINTS' not in server.commands()
    finally:
        driver.disconnect()


def test_emergency_stop_blocks_motion(server):
    driver = make_driver(server)
    try:
        assert driver.connect(), driver.last_error
        driver.emergency_stop()
        assert not driver.set_joint_positions([0.1, 0.3, -1.3, 0.0, 0.0, 0.0])
        assert 'MOVE_JOINTS' not in server.commands()
        assert 'SET_LEARNING_MODE' in server.commands()
        assert ('SET_LEARNING_MODE', ['TRUE']) not in server.requests  # never drop the arm
    finally:
        driver.disconnect()


def test_command_ko_keeps_the_link(server):
    driver = make_driver(server)
    try:
        assert driver.connect(), driver.last_error
        server.ko_on = 'MOVE_JOINTS'
        assert not driver.set_joint_positions([0.1, 0.3, -1.3, 0.0, 0.0, 0.0])
        assert driver.is_connected
        assert 'Command KO' in driver.last_error
        server.ko_on = None
        assert driver.set_joint_positions([0.1, 0.3, -1.3, 0.0, 0.0, 0.0]), driver.last_error
    finally:
        driver.disconnect()


def test_link_dropped_during_motion(server):
    driver = make_driver(server)
    try:
        assert driver.connect(), driver.last_error
        server.drop_on = 'MOVE_JOINTS'
        assert not driver.set_joint_positions([0.1, 0.3, -1.3, 0.0, 0.0, 0.0])
        assert not driver.is_connected
        assert 'Unable to communicate' in driver.last_error
        assert not driver.is_moving
        assert driver.get_joint_positions() == HOME  # last known positions, no exception
    finally:
        driver.disconnect()


def test_connection_refused_is_reported_without_hanging():
    # Reserve a free port, then close it so that nothing listens there.
    probe = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    probe.bind(('127.0.0.1', 0))
    port = probe.getsockname()[1]
    probe.close()

    def factory():
        robot = pyniryo.NiryoRobot()
        robot._NiryoRobot__port = port
        return robot

    driver = Ned1ArmDriver(ip_address='127.0.0.1', client_factory=factory)
    assert not driver.connect()
    assert not driver.is_connected
    assert 'Unable to connect' in driver.last_error
