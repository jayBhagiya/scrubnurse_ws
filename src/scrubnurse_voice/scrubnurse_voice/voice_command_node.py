"""
Voice commands for the Scrubnurse: speech to tool id with Whisper.

Publishes the recognised tool id (e.g. "D") on `tool_request` (std_msgs/String),
every transcript on `~/transcript`, and what happened to it (heard, wake word,
matched tool; microphone errors) on `~/events` (scrubnurse_interfaces/VoiceEvent).

With `live` the microphone is always on: each phrase (speech up to a pause) is
transcribed, and a tool is requested only if the phrase also contains a wake
word ("robot, scalpel"). Otherwise a fixed-length recording is started by the
GUI button (use_gui:=true, like the ROS 1 version) or the `~/listen` service
(std_srvs/Trigger). `audio_file` transcribes one file at startup, for testing
without a microphone.
"""

import queue
import threading

import rclpy
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from scrubnurse_interfaces.msg import VoiceEvent
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
        input_device = self.declare_parameter('input_device', '').value
        self._input_device = input_device or None
        self.use_gui = self.declare_parameter('use_gui', True).value
        self.live = self.declare_parameter('live', False).value
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
        self.results = queue.Queue()  # (text, tool_id or None) for the GUI

        self._request_pub = self.create_publisher(String, 'tool_request', 10)
        self._transcript_pub = self.create_publisher(String, '~/transcript', 10)
        self._event_pub = self.create_publisher(VoiceEvent, '~/events', 10)
        if self.live:
            threading.Thread(target=self._listen_live, daemon=True).start()
        else:
            self.create_service(Trigger, '~/listen', self._on_listen)
        if audio_file:
            self._file_timer = self.create_timer(0.5, lambda: self._from_file(audio_file))
        self.get_logger().info('Ready')

    def listen(self):
        """Record from the microphone, transcribe and publish; return (text, tool_id)."""
        self.get_logger().info(f'Recording {self.record_seconds:.0f} s...')
        return self.process(audio.record(self.record_seconds, self._input_device))

    def process(self, samples, need_wake_word=False):
        """Transcribe 16 kHz audio and publish the result; return (text, tool_id)."""
        with self._busy:
            text = self._model.transcribe(samples, language='en', fp16=self._fp16)['text']
        self._transcript_pub.publish(String(data=text))
        if need_wake_word and not has_wake_word(text, self.wake_words, self._wake_threshold):
            self.get_logger().info(f'"{text.strip()}": no wake word, ignored')
            self.publish_event('heard', text.strip())
            return text, None
        tool_id = self._catalog.match(text)
        self.publish_event('heard', text.strip(), wake_word=True, tool_id=tool_id or '')
        if self.use_gui:
            self.results.put((text, tool_id))
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

    def _listen_live(self):
        self.get_logger().info(f'Listening; say e.g. "{self.wake_words[0]}, scalpel"')
        self.publish_event('ready', '')
        try:
            for samples in audio.phrases(rclpy.ok, self._speech_level, self._end_silence,
                                         device=self._input_device):
                self.process(samples, need_wake_word=True)
        except Exception as e:  # e.g. the microphone was unplugged
            if rclpy.ok():
                self.get_logger().error(f'live listening stopped: {e}')
                self.publish_event('error', f'live listening stopped: {e}')
                self.results.put((f'Error: {e}', None))

    def _from_file(self, path):
        self._file_timer.cancel()
        self.process(audio.load_audio_file(path))

    def _on_listen(self, request, response):
        text, tool_id = self.listen()
        response.success = tool_id is not None
        response.message = f'"{text.strip()}" -> {tool_id}'
        return response


class RecorderWindow:
    """Small Tk window with one record button, like the ROS 1 'Audio Recorder'."""

    def __init__(self, node):
        import tkinter as tk

        self._node = node
        self.root = tk.Tk()
        self.root.title('Scrubnurse voice command')
        self.root.geometry('500x300')
        self.root.report_callback_exception = self._on_callback_error
        self._button = tk.Button(self.root, text='Start Recording', command=self._start,
                                 bg='green', fg='white', width=20, height=3)
        if node.live:
            self._button.config(text=f'Listening for "{node.wake_words[0]}"', state='disabled')
        self._button.place(relx=0.5, rely=0.4, anchor=tk.CENTER)
        self._label = tk.Label(self.root, text='', font=('Helvetica', 16), wraplength=460)
        self._label.place(relx=0.5, rely=0.8, anchor=tk.CENTER)
        self.root.after(100, self._poll)

    def run(self):
        self.root.mainloop()

    def _on_callback_error(self, exc, value, traceback):
        # Tk swallows a Ctrl-C raised inside a callback; close the window instead.
        if issubclass(exc, KeyboardInterrupt):
            self.root.destroy()
        else:
            type(self.root).report_callback_exception(self.root, exc, value, traceback)

    def _start(self):
        self._button.config(text='Recording', state='disabled', bg='red')
        threading.Thread(target=self._listen, daemon=True).start()
        self._countdown(int(self._node.record_seconds))

    def _listen(self):
        try:
            self._node.listen()
        except Exception as e:  # microphone or Whisper failure; the button must come back
            self._node.get_logger().error(f'recording failed: {e}')
            self._node.publish_event('error', f'recording failed: {e}')
            self._node.results.put((f'Error: {e}', None))

    def _countdown(self, remaining):
        if str(self._button['state']) != 'disabled':
            return
        if remaining > 0:
            self._label.config(text=f'Recording... {remaining}')
            self.root.after(1000, self._countdown, remaining - 1)
        else:
            self._label.config(text='Processing...')

    def _poll(self):
        if not rclpy.ok():
            self.root.destroy()
            return
        try:
            text, tool_id = self._node.results.get_nowait()
            if not self._node.live:
                self._button.config(text='Start Recording', state='normal', bg='green')
            self._label.config(text=f'{text.strip()}  ->  {tool_id or "no tool matched"}')
        except queue.Empty:
            pass
        self.root.after(100, self._poll)


def spin_until_shutdown(executor):
    try:
        executor.spin()
    except ExternalShutdownException:  # Ctrl-C: rclpy shut down under the GUI thread
        pass


def main(args=None):
    rclpy.init(args=args)
    node = VoiceCommandNode()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        if node.use_gui:
            threading.Thread(target=spin_until_shutdown, args=(executor,), daemon=True).start()
            RecorderWindow(node).run()
        else:
            executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
