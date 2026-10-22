"""
Voice commands for the Scrubnurse: speech to tool id with Whisper.

Publishes the recognised tool id (e.g. "D") on `tool_request` (std_msgs/String)
and every transcript on `~/transcript`. A recording is started by the GUI
button (use_gui:=true, like the ROS 1 version), by calling the `~/listen`
service (std_srvs/Trigger), or once at startup from `audio_file` for testing
without a microphone.
"""

import queue
import threading

import rclpy
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from scrubnurse_voice import audio
from scrubnurse_voice.tool_catalog import DEFAULT_TOOL_IDS, DEFAULT_TOOL_NAMES, ToolCatalog
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
        audio_file = self.declare_parameter('audio_file', '').value
        self._catalog = ToolCatalog(
            self.declare_parameter('tool_ids', DEFAULT_TOOL_IDS).value,
            self.declare_parameter('tool_names', DEFAULT_TOOL_NAMES).value,
            self.declare_parameter('similarity_threshold', 0.75).value)

        import torch
        import whisper

        if device.startswith('cuda') and not torch.cuda.is_available():
            self.get_logger().warn('CUDA not available, running Whisper on CPU')
            device = 'cpu'
        self._fp16 = device != 'cpu'
        self.get_logger().info(f'Loading Whisper "{model_name}" on {device}...')
        self._model = whisper.load_model(model_name, device=device)
        self._busy = threading.Lock()

        self._request_pub = self.create_publisher(String, 'tool_request', 10)
        self._transcript_pub = self.create_publisher(String, '~/transcript', 10)
        self.create_service(Trigger, '~/listen', self._on_listen)
        if audio_file:
            self._file_timer = self.create_timer(0.5, lambda: self._from_file(audio_file))
        self.get_logger().info('Ready')

    def listen(self):
        """Record from the microphone, transcribe and publish; return (text, tool_id)."""
        self.get_logger().info(f'Recording {self.record_seconds:.0f} s...')
        return self.process(audio.record(self.record_seconds, self._input_device))

    def process(self, samples):
        """Transcribe 16 kHz audio and publish the result; return (text, tool_id)."""
        with self._busy:
            text = self._model.transcribe(samples, language='en', fp16=self._fp16)['text']
        tool_id = self._catalog.match(text)
        self._transcript_pub.publish(String(data=text))
        if tool_id is None:
            self.get_logger().warn(f'"{text.strip()}": no tool matched')
        else:
            self.get_logger().info(f'"{text.strip()}" -> tool {tool_id}')
            self._request_pub.publish(String(data=tool_id))
        return text, tool_id

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
        self._results = queue.Queue()
        self.root = tk.Tk()
        self.root.title('Scrubnurse voice command')
        self.root.geometry('500x300')
        self._button = tk.Button(self.root, text='Start Recording', command=self._start,
                                 bg='green', fg='white', width=20, height=3)
        self._button.place(relx=0.5, rely=0.4, anchor=tk.CENTER)
        self._label = tk.Label(self.root, text='', font=('Helvetica', 16), wraplength=460)
        self._label.place(relx=0.5, rely=0.8, anchor=tk.CENTER)
        self.root.after(100, self._poll)

    def run(self):
        self.root.mainloop()

    def _start(self):
        self._button.config(text='Recording', state='disabled', bg='red')
        threading.Thread(target=lambda: self._results.put(self._node.listen()),
                         daemon=True).start()
        self._countdown(int(self._node.record_seconds))

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
            text, tool_id = self._results.get_nowait()
            self._button.config(text='Start Recording', state='normal', bg='green')
            self._label.config(text=f'{text.strip()}  ->  {tool_id or "no tool matched"}')
        except queue.Empty:
            pass
        self.root.after(100, self._poll)


def main(args=None):
    rclpy.init(args=args)
    node = VoiceCommandNode()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        if node.use_gui:
            threading.Thread(target=executor.spin, daemon=True).start()
            RecorderWindow(node).run()
        else:
            executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
