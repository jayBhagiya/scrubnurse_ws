"""
HTTP side of the dashboard: REST controls, the state WebSocket and the camera stream.

`node` is the DashboardNode (or a stand-in in the tests) and provides snapshot(),
health(), refusal(action), call(action), request_tool(tool_id), jpeg(), log() and
tool_names.
"""

import asyncio
import os

from fastapi import Body, FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.responses import StreamingResponse
from fastapi.staticfiles import StaticFiles

STATE_PERIOD = 0.1   # s, WebSocket updates (10 Hz)
STREAM_PERIOD = 0.1  # s, camera stream frames at most


def create_app(node, web_dir=None):
    app = FastAPI(title='Scrubnurse dashboard')

    def control(action, call):
        reason = node.refusal(action)
        if reason:
            node.log('warn', 'dashboard', f'{action} refused: {reason}')
            raise HTTPException(409, reason)
        try:
            ok, message = call()
        except TimeoutError as e:
            node.log('error', 'dashboard', f'{action}: {e}')
            raise HTTPException(504, str(e))
        if not ok:
            node.log('warn', 'dashboard', f'{action} refused by the controller: {message}')
            raise HTTPException(409, message)
        node.log('info', 'dashboard', f'{action}: {message}' if message else action)
        return {'ok': True, 'message': message}

    @app.get('/api/state')
    def state():
        return node.snapshot()

    @app.get('/api/health')
    def health():
        return node.health()

    @app.post('/api/pause')
    def pause():
        return control('pause', lambda: node.call('pause'))

    @app.post('/api/resume')
    def resume():
        return control('resume', lambda: node.call('resume'))

    @app.post('/api/abort')
    def abort():
        return control('abort', lambda: node.call('abort'))

    @app.post('/api/recover')
    def recover(arm_clear: bool = Body(False, embed=True)):
        if not arm_clear:  # recover moves the arm: the operator must confirm it is clear
            raise HTTPException(400, 'confirm that the arm is clear (arm_clear: true)')
        return control('recover', lambda: node.call('recover'))

    @app.post('/api/request')
    def request(instrument: str = Body(..., embed=True)):
        by_name = {name.lower(): tool_id for tool_id, name in node.tool_names.items()}
        key = instrument.strip()
        tool_id = key.upper() if key.upper() in node.tool_names else by_name.get(key.lower())
        if tool_id is None:
            raise HTTPException(422, f'unknown instrument "{instrument}"; one of: '
                                     + ', '.join(node.tool_names.values()))
        return control('request', lambda: node.request_tool(tool_id))

    @app.websocket('/ws/state')
    async def state_socket(socket: WebSocket):
        await socket.accept()
        try:
            while True:
                await socket.send_json(node.snapshot())
                await asyncio.sleep(STATE_PERIOD)
        except WebSocketDisconnect:
            pass

    @app.get('/stream/camera')
    async def camera():
        async def frames():
            last = None
            while True:
                jpeg = await asyncio.to_thread(node.jpeg)  # encodes the newest frame once
                if jpeg is not None and jpeg is not last:
                    last = jpeg
                    yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + jpeg + b'\r\n')
                await asyncio.sleep(STREAM_PERIOD)
        return StreamingResponse(frames(), media_type='multipart/x-mixed-replace; boundary=frame')

    if web_dir and os.path.isdir(web_dir):
        app.mount('/', StaticFiles(directory=web_dir, html=True), name='web')
    return app
