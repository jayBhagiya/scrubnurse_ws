from fastapi.testclient import TestClient
import pytest
from scrubnurse_dashboard.app import create_app
from scrubnurse_dashboard.state import refusal


class FakeNode:
    tool_names = {'B': 'needle holder', 'D': 'scalpel'}

    def __init__(self):
        self.cycle = {'paused': True, 'aborted': False, 'error': ''}
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


@pytest.fixture
def node():
    return FakeNode()


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
