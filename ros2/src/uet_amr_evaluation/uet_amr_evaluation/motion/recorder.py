# Copyright (c) 2026 Department of Robotics, University of Engineering and
#                    Technology, Vietnam National University, Hanoi (VNU).
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to
# deal in the Software without restriction, including without limitation the
# rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
# sell copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
# FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
# DEALINGS IN THE SOFTWARE.

"""
Robot-side run recorder: native-rate CDR captured in the subscription callbacks (before any network forwarding),
handed through a bounded queue to a disk worker, and exported as the dashboard's JSON archive.

On disk, per run, in <dir>:
  <run_id>.cdrlog   magic + length-prefixed JSON header (topic table) + records
                    (<HQQI> topic index, robot wall ns, robot steady ns, length) + CDR bytes
  <run_id>.meta.json  status "capturing" until the worker finishes, then "complete" | "incomplete"
A file whose meta still says "capturing" was interrupted; it is kept and never exported as complete.
Nothing here ever resumes a run.
"""
import base64
import json
import os
import queue
import re
import struct
import threading
import time
import uuid

MAGIC = b'AMRCDR1\n'
RECORD = struct.Struct('<HQQI')
MAX_PACKETS = 20_000
MAX_BYTES = 16 * 1024 * 1024
RUN_ID_RE = re.compile(r'[A-Za-z0-9][A-Za-z0-9_.-]{0,127}')


def valid_run_id(run_id):
    return bool(RUN_ID_RE.fullmatch(run_id)) and '..' not in run_id


def _paths(directory, run_id):
    return os.path.join(directory, run_id + '.cdrlog'), os.path.join(directory, run_id + '.meta.json')


def run_exists(directory, run_id):
    return any(os.path.exists(p) for p in _paths(directory, run_id))


def _write_json(path, obj):
    tmp = path + '.tmp'
    with open(tmp, 'w', encoding='utf-8') as f:
        json.dump(obj, f, indent=2)
        f.flush()
        os.fsync(f.fileno())
    os.replace(tmp, path)


class RunRecorder:
    """One capture. record() is called from ROS callbacks and never blocks."""

    def __init__(self, directory, run_id, topics, queue_size=4096, snapshot=None):
        os.makedirs(directory, exist_ok=True)
        self.run_id, self.topics = run_id, list(topics)
        self._log_path, self._meta_path = _paths(directory, run_id)
        self._queue = queue.Queue(maxsize=queue_size)
        self._order_lock = threading.Lock()
        self._active = False
        self.dropped = 0
        self.count = self.bytes = 0
        self.error = ''
        self._meta = {
            'schema_version': 1, 'run_id': run_id, 'archive_id': str(uuid.uuid4()), 'status': 'capturing',
            'topics': self.topics, 'started_wall_ns': str(time.time_ns()), 'parameter_snapshot': snapshot or {},
        }
        self._file = open(self._log_path, 'xb')       # 'x': never overwrite an earlier run
        header = json.dumps({'topics': self.topics}).encode()
        self._file.write(MAGIC + struct.pack('<I', len(header)) + header)
        self._file.flush()
        _write_json(self._meta_path, self._meta)
        self._worker = threading.Thread(target=self._drain, name=f'recorder-{run_id}', daemon=True)
        self._worker.start()
        self._active = True

    def record(self, topic, cdr):
        """Timestamp and enqueue under one lock so queue order is robot receipt order."""
        if not self._active:
            return
        with self._order_lock:
            if not self._active:
                return
            item = (self.topics.index(topic), time.time_ns(), time.monotonic_ns(), bytes(cdr))
            try:
                self._queue.put_nowait(item)
            except queue.Full:
                self.dropped += 1

    def finish(self):
        """Stop accepting samples and let the worker drain; returns immediately."""
        with self._order_lock:
            if not self._active:
                return
            self._active = False
            self._queue.put(None)                 # sentinel; the worker is draining so this cannot deadlock for long

    def join(self, timeout=10.0):
        self._worker.join(timeout)

    def _drain(self):
        last_flush = time.monotonic()
        try:
            while True:
                try:
                    item = self._queue.get(timeout=0.2)
                except queue.Empty:
                    item = False
                if item is None:
                    break
                if item:
                    idx, wall, mono, cdr = item
                    if self.count >= MAX_PACKETS or self.bytes + len(cdr) > MAX_BYTES:
                        self.error = 'archive size limit exceeded'
                        self.dropped += 1
                        continue
                    self._file.write(RECORD.pack(idx, wall, mono, len(cdr)) + cdr)
                    self.count += 1
                    self.bytes += len(cdr)
                if time.monotonic() - last_flush > 0.25:
                    self._file.flush()
                    last_flush = time.monotonic()
        except OSError as exc:
            self.error = f'disk error: {exc}'
        finally:
            try:
                self._file.flush()
                os.fsync(self._file.fileno())
                self._file.close()
            except OSError as exc:
                self.error = self.error or f'disk error: {exc}'
            complete = self.dropped == 0 and not self.error
            self._meta.update(status='complete' if complete else 'incomplete', count=self.count,
                              decoded_bytes=self.bytes, dropped=self.dropped, error=self.error,
                              finished_wall_ns=str(time.time_ns()))
            try:
                _write_json(self._meta_path, self._meta)
            except OSError:
                pass


class ArchiveError(Exception):
    pass


def read_records(log_path):
    """Yield (topic, wall_ns, mono_ns, cdr) from a log; a truncated tail record is ignored."""
    with open(log_path, 'rb') as f:
        if f.read(len(MAGIC)) != MAGIC:
            raise ArchiveError('not a recorder log')
        (hlen,) = struct.unpack('<I', f.read(4))
        topics = json.loads(f.read(hlen))['topics']
        while True:
            head = f.read(RECORD.size)
            if len(head) < RECORD.size:
                return
            idx, wall, mono, n = RECORD.unpack(head)
            cdr = f.read(n)
            if len(cdr) < n:
                return
            yield topics[idx], wall, mono, cdr


def export_archive(directory, run_id):
    """Build the dashboard JSON archive dict for a finished, complete capture; raises ArchiveError otherwise."""
    if not valid_run_id(run_id):
        raise ArchiveError('invalid run_id')
    log_path, meta_path = _paths(directory, run_id)
    try:
        with open(meta_path, encoding='utf-8') as f:
            meta = json.load(f)
    except (OSError, ValueError) as exc:
        raise ArchiveError(f'no capture metadata for {run_id}: {exc}')
    if meta.get('status') != 'complete':
        raise ArchiveError(
            f'capture for {run_id} is {meta.get("status")} (dropped={meta.get("dropped")}, '
            f'error={meta.get("error")!r}); refusing to label it complete')
    packets = [{'topic': t, 'robot_receipt_ns': str(w), 'robot_monotonic_ns': str(m),
                'cdr_base64': base64.b64encode(c).decode('ascii')} for t, w, m, c in read_records(log_path)]
    if len(packets) != meta['count']:
        raise ArchiveError(f'log holds {len(packets)} packets but {meta["count"]} were recorded')
    return {'schema_version': 1, 'run_id': run_id, 'archive_id': meta['archive_id'],
            'expected_count': len(packets), 'packets': packets}


def load_snapshot(directory, run_id):
    with open(_paths(directory, run_id)[1], encoding='utf-8') as f:
        return json.load(f).get('parameter_snapshot', {})
