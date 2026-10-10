"""Bounded, serial JSONL calls to one locally owned map/planner process."""
from sim_io import json_codec as json
import os
import select
import subprocess
import threading
import time


class JsonProcess:
    def __init__(self, command, stderr_path, timeout=20):
        self.log = open(stderr_path, 'ab', buffering=0)
        self.process = subprocess.Popen(command, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                        stderr=self.log, bufsize=0)
        self.lock = threading.Lock()
        self.buffer = b''
        self.timeout = timeout

    def call(self, request, timeout=None):
        with self.lock:
            if self.process.poll() is not None:
                raise RuntimeError('worker exited: ' + str(self.process.returncode))
            raw = json.dumps(request, separators=(',', ':'), allow_nan=False).encode() + b'\n'
            remaining = memoryview(raw)
            while remaining:
                count = self.process.stdin.write(remaining)
                if not count:raise RuntimeError('worker stdin closed')
                remaining = remaining[count:]
            deadline = time.monotonic() + (self.timeout if timeout is None else timeout)
            while b'\n' not in self.buffer:
                remaining = deadline - time.monotonic()
                if remaining <= 0 or not select.select([self.process.stdout], [], [], remaining)[0]:
                    self.process.terminate()
                    self.process.wait(timeout=5)
                    raise TimeoutError('worker response deadline; terminated to reject late response')
                part = os.read(self.process.stdout.fileno(), 1024 * 1024)
                if not part:
                    raise RuntimeError('worker stdout closed')
                self.buffer += part
                if len(self.buffer) > 160 * 1024 * 1024:
                    raise RuntimeError('worker response exceeds bounded snapshot size')
            line, self.buffer = self.buffer.split(b'\n', 1)
            result = json.loads(line)
            return result

    def close(self):
        if self.process.poll() is None:
            self.process.terminate()
            try:
                self.process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(timeout=5)
        self.log.close()
