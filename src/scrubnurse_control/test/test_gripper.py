import pytest
from scrubnurse_control.gripper import RobotiqSocketGripper


class FakeSocket:

    def __init__(self, reply):
        self._reply = reply

    def sendall(self, data):
        pass

    def recv(self, size):
        return self._reply


def gripper_replying(reply):
    gripper = RobotiqSocketGripper('localhost')
    gripper._socket = FakeSocket(reply)
    return gripper


def test_get_reads_the_value():
    assert gripper_replying(b'POS 128\n')._get('POS') == 128


@pytest.mark.parametrize('reply', [b'', b'ack', b'STA 3', b'POS ?', b'\xff\xfe'])
def test_bad_reply_is_a_connection_error(reply):
    # ConnectionError is an OSError, which the controller catches and pauses on.
    with pytest.raises(ConnectionError):
        gripper_replying(reply)._get('POS')
