"""
Voice commands for the Scrubnurse: speech to tool id with Whisper.

Publishes the recognised tool id (e.g. "D") on `tool_request` (std_msgs/String),
every transcript on `~/transcript`, what happened to it (heard, wake word,
matched tool; microphone errors) on `~/events` (scrubnurse_interfaces/VoiceEvent)
and the node's state on `~/status` (VoiceStatus, latched; the dashboard shows it).

With `live` the microphone is always on: each phrase (speech up to a pause) is
transcribed, and a tool is requested only if the phrase also contains a wake
word ("robot, scalpel"). `~/listen` (std_srvs/Trigger) then accepts the next
phrase without a wake word (push-to-talk). Without `live`, `~/listen` records
`record_seconds` of audio. The parameters `muted` and `input_device` can be
changed at runtime; a new microphone restarts the stream. `audio_file`
transcribes one file at startup, for testing without a microphone.
"""

import threading
import time

from rcl_interfaces.msg import SetParametersResult
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from scrubnurse_interfaces.msg import VoiceEvent, VoiceStatus
from scrubnurse_voice import audio
from scrubnurse_voice.tool_catalog import DEFAULT_TOOL_IDS, DEFAULT_TOOL_NAMES
from scrubnurse_voice.tool_catalog import has_wake_word, ToolCatalog
from std_msgs.msg import String
from std_srvs.srv import Trigger


class VoiceCommandNode(Node):

    def __init__(self):
        super().__init__('voice_command')
        model_name = self.declare_parameter('model', 'medium.en').value
        device = self.declare_parameter('device', 'cuda').value
        self.record_seconds = self.declare_parameter('record_seconds', 5.0).value
        self._input_device = self.declare_parameter('input_device', '').value or None
        self.live = self.declare_parameter('live', False).value
        self._muted = self.declare_parameter('muted', False).value
        self.wake_words = self.declare_parameter(
            'wake_words', ['robot', 'scrub nurse', 'scrubnurse']).value
        self._wake_threshold = self.declare_parameter('wake_threshold', 0.65).value
        self._speech_level = self.declare_parameter('speech_level', 0.01).value
        self._end_silence = self.declare_parameter('end_silence', 0.8).value
        audio_file = self.declare_parameter('audio_file', '').value
        tool_ids = self.declare_parameter('tool_ids', DEFAULT_TOOL_IDS).value
        tool_names = self.declare_parameter('tool_names', DEFAULT_TOOL_NAMES).value
        self._catalog = ToolCatalog(tool_ids, tool_names,
                                    self.declare_parameter('similarity_threshold', 0.75).value)
        self._tool_names = dict(zip(tool_ids, tool_names))

        import torch
        import whisper

        if device.startswith('cuda') and not torch.cuda.is_available():
            self.get_logger().warn('CUDA not available, running Whisper on CPU')
            device = 'cpu'
        self._fp16 = device != 'cpu'
        self.get_logger().info(f'Loading Whisper "{model_name}" on {device}...')
        self._model = whisper.load_model(model_name, device=device)
        self._busy = threading.Lock()

        # shortcut: the list is read once; PortAudio cannot rescan while a stream is open, so a
        # microphone plugged in later appears after a restart of the node.
        self._devices = audio.input_devices()
        self._state, self._message = '', ''
        self._command_until = 0.0  # push-to-talk: until then (monotonic) no wake word needed
        self._generation = 0       # bumped by a new input_device; restarts the live stream

        self._request_pub = self.create_publisher(String, 'tool_request', 10)
        self._transcript_pub = self.create_publisher(String, '~/transcript', 10)
        self._event_pub = self.create_publisher(VoiceEvent, '~/events', 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._status_pub = self.create_publisher(VoiceStatus, '~/status', latched)
        self.add_on_set_parameters_callback(self._on_parameters)
        self.create_service(Trigger, '~/listen', self._on_listen)
        self.create_timer(0.5, self._end_command_window)
        self._set_state(self._resting_state())
        if self.live:
            threading.Thread(target=self._listen_live, daemon=True).start()
        if audio_file:
            self._file_timer = self.create_timer(0.5, lambda: self._from_file(audio_file))
        self.get_logger().info('Ready')

    def process(self, samples, need_wake_word=False):
        """Transcribe 16 kHz audio and publish the result; return (text, tool_id)."""
        self._set_state(VoiceStatus.STATE_PROCESSING)
        try:
            with self._busy:
                text = self._model.transcribe(samples, language='en', fp16=self._fp16)['text']
        finally:
            self._set_state(self._resting_state())
        self._transcript_pub.publish(String(data=text))
        if not any(c.isalpha() for c in text):  # Whisper on noise: '', '...', '♪'
            return text, None
        if need_wake_word and not has_wake_word(text, self.wake_words, self._wake_threshold):
            self.get_logger().info(f'"{text.strip()}": no wake word, ignored')
            self.publish_event('heard', text.strip())
            return text, None
        tool_id = self._catalog.match(text)
        self.publish_event('heard', text.strip(), wake_word=True, tool_id=tool_id or '')
        if tool_id is None:
            self.get_logger().warn(f'"{text.strip()}": no tool matched')
        else:
            self.get_logger().info(f'"{text.strip()}" -> tool {tool_id}')
            self._request_pub.publish(String(data=tool_id))
        return text, tool_id

    def publish_event(self, kind, text, wake_word=False, tool_id=''):
        self._event_pub.publish(VoiceEvent(
            stamp=self.get_clock().now().to_msg(), kind=kind, text=text, wake_word=wake_word,
            tool_id=tool_id, tool_name=self._tool_names.get(tool_id, '')))

    def _resting_state(self):
        if not self.live:
            return VoiceStatus.STATE_IDLE
        if time.monotonic() < self._command_until:
            return VoiceStatus.STATE_COMMAND
        return VoiceStatus.STATE_MUTED if self._muted else VoiceStatus.STATE_LISTENING

    def _set_state(self, state, message=''):
        self._state, self._message = state, message
        self._status_pub.publish(VoiceStatus(
            stamp=self.get_clock().now().to_msg(), state=state, message=message, live=self.live,
            muted=self._muted, input_device=self._input_device or '', devices=self._devices,
            wake_words=self.wake_words))

    def _on_parameters(self, params):
        for p in params:
            if p.name == 'input_device':
                self._input_device = p.value or None
                self._generation += 1
                self.get_logger().info(f'microphone: {self._input_device or "system default"}')
            elif p.name == 'muted':
                self._muted = bool(p.value)
            elif p.name in ('live', 'model', 'device'):
                return SetParametersResult(successful=False, reason=f'{p.name}: restart the node')
        if self._state != VoiceStatus.STATE_PROCESSING:
            self._set_state(self._resting_state(), self._message)
        return SetParametersResult(successful=True)

    def _end_command_window(self):
        if self._state == VoiceStatus.STATE_COMMAND and time.monotonic() >= self._command_until:
            self._set_state(self._resting_state())

    def _listen_live(self):
        while rclpy.ok():
            generation = self._generation
            self.get_logger().info(f'Listening; say e.g. "{self.wake_words[0]}, scalpel"')
            self._set_state(self._resting_state())
            self.publish_event('ready', '')
            try:
                for samples in audio.phrases(
                        lambda: rclpy.ok() and self._generation == generation,
                        self._speech_level, self._end_silence, device=self._input_device):
                    command = time.monotonic() < self._command_until
                    if self._muted and not command:
                        continue
                    self._command_until = 0.0
                    self.process(samples, need_wake_word=not command)
            except Exception as e:  # e.g. the microphone was unplugged
                if not rclpy.ok():
                    return
                self.get_logger().error(f'live listening stopped: {e}')
                self.publish_event('error', f'live listening stopped: {e}')
                self._set_state(VoiceStatus.STATE_ERROR, str(e))
                end = time.monotonic() + 5.0  # retry, or sooner with another microphone
                while rclpy.ok() and self._generation == generation and time.monotonic() < end:
                    time.sleep(0.1)

    def _from_file(self, path):
        self._file_timer.cancel()
        self.process(audio.load_audio_file(path))

    def _on_listen(self, request, response):
        if self.live:
            self._command_until = time.monotonic() + self.record_seconds
            self._set_state(VoiceStatus.STATE_COMMAND)
            response.success = True
            response.message = f'say the command within {self.record_seconds:.0f} s'
            return response
        self._set_state(VoiceStatus.STATE_COMMAND)
        try:
            self.get_logger().info(f'Recording {self.record_seconds:.0f} s...')
            text, tool_id = self.process(audio.record(self.record_seconds, self._input_device))
        except Exception as e:  # microphone or Whisper failure
            self.get_logger().error(f'recording failed: {e}')
            self.publish_event('error', f'recording failed: {e}')
            self._set_state(VoiceStatus.STATE_ERROR, str(e))
            response.success, response.message = False, str(e)
            return response
        response.success = tool_id is not None
        response.message = f'"{text.strip()}" -> {tool_id}'
        return response


def main(args=None):
    rclpy.init(args=args)
    node = VoiceCommandNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
