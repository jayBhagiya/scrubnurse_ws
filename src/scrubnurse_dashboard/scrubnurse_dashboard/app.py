"""
HTTP side of the dashboard: REST controls, the state WebSocket and the camera stream.

`node` is the DashboardNode (or a stand-in in the tests) and provides snapshot(),
replay(action, value) for the replay timeline,
health(), refusal(action), call(action), request_tool(tool_id), jpeg(), log() and
tool_names; scene_jpeg() for the simulation's side view; for voice and camera voice_status(),
camera_status(), voice_listen() and set_parameter(node, name, value); for the calibration page
also `calibration`
(calibration.Calibration), block_now(), block_pixel(), tool0() and image_size().
"""

import asyncio
import os
import time

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

    def node_call(source, action, call):
        """Run a call to the voice or camera node: 504 on timeout, 409 when it refuses."""
        try:
            ok, message = call()
        except TimeoutError as e:
            node.log('error', source, f'{action}: {e}')
            raise HTTPException(504, str(e))
        if not ok:
            node.log('warn', source, f'{action} refused: {message}')
            raise HTTPException(409, message or 'refused')
        node.log('info', source, f'{action}: {message}' if message else action)
        return {'ok': True, 'message': message}

    def running_voice():
        status = node.voice_status()
        if status is None:
            raise HTTPException(409, 'the voice node is not running')
        return status

    @app.post('/api/voice/listen')
    def voice_listen():
        running_voice()
        return node_call('voice', 'push-to-talk', node.voice_listen)

    @app.post('/api/voice/mute')
    def voice_mute(muted: bool = Body(..., embed=True)):
        if not running_voice()['live']:
            raise HTTPException(409, 'the voice node only listens on push-to-talk (live: false)')
        return node_call('voice', 'mute' if muted else 'unmute',
                         lambda: node.set_parameter('/voice_command', 'muted', muted))

    @app.post('/api/voice/microphone')
    def voice_microphone(name: str = Body(..., embed=True)):
        if name and name not in running_voice()['devices']:
            raise HTTPException(422, f'unknown microphone "{name}"')
        return node_call('voice', f'microphone: {name or "system default"}',
                         lambda: node.set_parameter('/voice_command', 'input_device', name))

    @app.post('/api/camera')
    def camera_select(serial: str = Body(..., embed=True)):
        status = node.camera_status()
        if status is None:
            raise HTTPException(409, 'the camera node is not running')
        if serial not in status['serials']:
            raise HTTPException(422, f'no Azure Kinect "{serial}" connected')
        return node_call('camera', f'switch to {serial}', lambda: node.set_parameter(
            '/kinect_camera', 'serial', serial, timeout=15.0))  # reopening takes a few seconds

    @app.post('/api/replay/pause')
    def replay_pause():
        return node_call('replay', 'pause', lambda: node.replay('pause'))

    @app.post('/api/replay/resume')
    def replay_resume():
        return node_call('replay', 'play', lambda: node.replay('resume'))

    @app.post('/api/replay/seek')
    def replay_seek(time: float = Body(..., embed=True, ge=0)):
        return node_call('replay', f'jump to {time:.1f} s', lambda: node.replay('seek', time))

    @app.post('/api/replay/rate')
    def replay_rate(rate: float = Body(..., embed=True, gt=0, le=10)):
        return node_call('replay', f'speed {rate:g}x', lambda: node.replay('set_rate', rate))

    def calibration_step(action, step):
        try:
            step()
        except ValueError as e:
            raise HTTPException(409, str(e))
        node.log('info', 'calibration', action)
        return calibration()

    @app.get('/api/calibration')
    def calibration():
        return dict(node.calibration.state(), target=node.block_now(), tool0=node.tool0(),
                    image_size=node.image_size())

    @app.post('/api/calibration/pixel')
    def calibration_pixel():
        return calibration_step('pixel position recorded', node.block_pixel)

    @app.delete('/api/calibration/pixel')
    def calibration_clear_pixel():
        return calibration_step('pixel position cleared', node.calibration.clear_pixel)

    @app.post('/api/calibration/reference')
    def calibration_reference():
        def record():
            tool0 = node.tool0()
            if tool0 is None:
                raise ValueError('no tool0 position (is the robot driver running?)')
            node.calibration.add_reference(tool0['x'], tool0['y'])
        return calibration_step('reference position recorded', record)

    @app.delete('/api/calibration/pairs/{index}')
    def calibration_remove(index: int):
        return calibration_step(f'pair {index + 1} removed',
                                lambda: node.calibration.remove(index))

    @app.post('/api/calibration/save')
    def calibration_save():
        try:
            paths = node.calibration.save()
        except ValueError as e:
            raise HTTPException(409, str(e))
        node.log('info', 'calibration', 'saved ' + ' and '.join(paths))
        return dict(calibration(), saved=paths)

    @app.post('/api/calibration/load')
    def calibration_load(name: str = Body(..., embed=True)):
        return calibration_step(f'loaded {name}', lambda: node.calibration.load(name))

    @app.websocket('/ws/state')
    async def state_socket(socket: WebSocket):
        await socket.accept()
        try:
            while True:
                await socket.send_json(node.snapshot())
                await asyncio.sleep(STATE_PERIOD)
        except WebSocketDisconnect:
            pass

    def stream(newest_jpeg):
        async def frames():
            last, sent = None, 0.0
            while True:
                jpeg = await asyncio.to_thread(newest_jpeg)  # encodes the newest frame once
                # Repeat an unchanged frame now and then: browsers show a frame only once the
                # next one starts, so a paused replay would otherwise stay black.
                if jpeg is not None and (jpeg is not last or time.monotonic() - sent > 1.0):
                    last, sent = jpeg, time.monotonic()
                    yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + jpeg + b'\r\n')
                await asyncio.sleep(STREAM_PERIOD)
        return StreamingResponse(frames(), media_type='multipart/x-mixed-replace; boundary=frame')

    @app.get('/stream/camera')
    async def camera():
        return stream(node.jpeg)

    @app.get('/stream/scene')
    async def scene():
        return stream(node.scene_jpeg)

    if web_dir and os.path.isdir(web_dir):
        # follow_symlink: colcon --symlink-install links the files to web/dist
        app.mount('/', StaticFiles(directory=web_dir, html=True, follow_symlink=True),
                  name='web')
    return app
