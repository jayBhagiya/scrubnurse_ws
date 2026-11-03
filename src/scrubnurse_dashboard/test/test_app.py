from fastapi.testclient import TestClient
import pytest
from scrubnurse_dashboard.app import create_app
from scrubnurse_dashboard.calibration import Calibration
from scrubnurse_dashboard.state import refusal


class FakeNode:
    tool_names = {'B': 'needle holder', 'D': 'scalpel'}

    def __init__(self, folder='/nonexistent'):
        self.cycle = {'paused': True, 'aborted': False, 'error': ''}
        self.calibration = Calibration(folder)
        self.block = (1100.0, 500.0)
        self.position = {'x': 0.05, 'y': -0.95}
        self.calls = []
        self.answer = (True, 'ok')
        self.logs = []

    def refusal(self, action):
        return refusal(action, self.cycle, True)

    def call(self, action):
        self.calls.append(action)
        if self.answer is None:
            raise TimeoutError('no answer')
        return self.answer

    def request_tool(self, tool_id):
        self.calls.append(tool_id)
        return True, 'requested'

    def snapshot(self):
        return {'cycle': self.cycle}

    def health(self):
        return {'node': True}

    def log(self, *entry):
        self.logs.append(entry)

    def jpeg(self):
        return None

    def block_now(self):
        return {'u': self.block[0], 'v': self.block[1], 'corners': []}

    def block_pixel(self):
        if self.block is None:
            raise ValueError('no dark block of about 44 px found')
        self.calibration.set_pixel(*self.block)

    def tool0(self):
        return self.position

    def image_size(self):
        return [1920, 1080]


@pytest.fixture
def node(tmp_path):
    return FakeNode(str(tmp_path))


@pytest.fixture
def client(node):
    return TestClient(create_app(node))


def test_resume(client, node):
    assert client.post('/api/resume').json() == {'ok': True, 'message': 'ok'}
    assert node.calls == ['resume']


def test_refused_by_dashboard(client, node):
    node.cycle['aborted'] = True
    response = client.post('/api/resume')
    assert response.status_code == 409
    assert 'recover' in response.json()['detail']
    assert node.calls == []


def test_refused_by_controller(client, node):
    node.answer = (False, 'not aborted')
    assert client.post('/api/abort').status_code == 409


def test_timeout(client, node):
    node.answer = None
    assert client.post('/api/abort').status_code == 504


def test_recover_needs_confirmation(client, node):
    node.cycle['aborted'] = True
    assert client.post('/api/recover', json={}).status_code == 400
    assert client.post('/api/recover', json={'arm_clear': True}).status_code == 200
    assert node.calls == ['recover']


@pytest.mark.parametrize('instrument, tool_id', [('D', 'D'), ('d', 'D'), ('Scalpel', 'D'),
                                                 ('needle holder', 'B')])
def test_request(client, node, instrument, tool_id):
    assert client.post('/api/request', json={'instrument': instrument}).status_code == 200
    assert node.calls == [tool_id]


@pytest.mark.parametrize('body', [{'instrument': 'hammer'}, {'instrument': 'A'}, {}])
def test_request_unknown(client, node, body):
    assert client.post('/api/request', json=body).status_code == 422
    assert node.calls == []


def test_state_and_socket(client):
    assert client.get('/api/health').json() == {'node': True}
    with client.websocket_connect('/ws/state') as socket:
        assert socket.receive_json()['cycle']['paused']


def test_calibration_pair(client, node):
    assert client.post('/api/calibration/reference').status_code == 409  # no pixel yet
    assert client.post('/api/calibration/pixel').json()['pending']['u'] == 1100.0
    state = client.post('/api/calibration/reference').json()
    assert state['pairs'] == [{'u': 1100.0, 'v': 500.0, 'x': 0.05, 'y': -0.95}]
    assert state['pending'] is None
    assert client.delete('/api/calibration/pairs/0').json()['pairs'] == []
    assert client.delete('/api/calibration/pairs/0').status_code == 409


def test_calibration_refusals(client, node):
    node.block = None
    response = client.post('/api/calibration/pixel')
    assert response.status_code == 409 and 'no dark block' in response.json()['detail']
    node.block, node.position = (1100.0, 500.0), None
    client.post('/api/calibration/pixel')
    assert 'tool0' in client.post('/api/calibration/reference').json()['detail']
    assert client.post('/api/calibration/save').status_code == 409  # fewer than 4 pairs


def test_calibration_save_and_load(client, node):
    for i, (u, v) in enumerate([(900, 320), (1350, 320), (900, 720), (1350, 720), (1120, 520)]):
        node.block, node.position = (u, v), {'x': 0.5 - v / 1000, 'y': 0.15 - u / 1000 + i * 1e-4}
        client.post('/api/calibration/pixel')
        client.post('/api/calibration/reference')
    saved = client.post('/api/calibration/save').json()
    assert saved['fit']['unseen_mean'] is not None and len(saved['saved']) == 2
    name = saved['files'][0]
    client.delete('/api/calibration/pairs/0')
    assert len(client.post('/api/calibration/load', json={'name': name}).json()['pairs']) == 5
    assert client.post('/api/calibration/load', json={'name': '/etc/passwd'}).status_code == 409
