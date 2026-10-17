import json
import os

import pytest

from uet_amr_evaluation.motion import export_archive as cli
from uet_amr_evaluation.motion.recorder import ArchiveError, RunRecorder, export_archive, valid_run_id

TOPICS = ['/odometry/filtered', '/amr/motion_test/state']


def test_round_trip_exact_and_ordered(tmp_path):
    rec = RunRecorder(str(tmp_path), 'run-1', TOPICS)
    payloads = [(TOPICS[i % 2], bytes([i % 256]) * (i + 1)) for i in range(300)]
    for topic, cdr in payloads:
        rec.record(topic, cdr)
    rec.finish()
    rec.join()
    archive = export_archive(str(tmp_path), 'run-1')
    assert archive['schema_version'] == 1 and archive['run_id'] == 'run-1'
    assert archive['expected_count'] == len(archive['packets']) == 300
    import base64
    assert [(p['topic'], base64.b64decode(p['cdr_base64'])) for p in archive['packets']] == payloads
    walls = [int(p['robot_receipt_ns']) for p in archive['packets']]
    assert walls == sorted(walls) and all(isinstance(p['robot_monotonic_ns'], str) for p in archive['packets'])
    assert export_archive(str(tmp_path), 'run-1') == archive            # stable archive_id -> idempotent re-upload


def test_queue_drop_makes_capture_incomplete_and_unexportable(tmp_path):
    rec = RunRecorder(str(tmp_path), 'run-2', TOPICS, queue_size=1)
    for _ in range(5000):
        rec.record(TOPICS[0], b'x' * 64)
    rec.finish()
    rec.join()
    if rec.dropped == 0:
        pytest.skip('worker kept up on this machine')
    with pytest.raises(ArchiveError):
        export_archive(str(tmp_path), 'run-2')
    assert json.load(open(tmp_path / 'run-2.meta.json'))['status'] == 'incomplete'


def test_packet_limit_marks_incomplete(tmp_path, monkeypatch):
    monkeypatch.setattr('uet_amr_evaluation.motion.recorder.MAX_PACKETS', 10)
    rec = RunRecorder(str(tmp_path), 'run-3', TOPICS)
    for _ in range(20):
        rec.record(TOPICS[0], b'a')
    rec.finish()
    rec.join()
    with pytest.raises(ArchiveError):
        export_archive(str(tmp_path), 'run-3')


def test_interrupted_capture_is_kept_but_not_exportable(tmp_path):
    rec = RunRecorder(str(tmp_path), 'run-4', TOPICS)
    rec.record(TOPICS[0], b'abc')
    import time
    time.sleep(0.5)                                  # crash: never finish()
    assert os.path.getsize(tmp_path / 'run-4.cdrlog') > 0
    with pytest.raises(ArchiveError, match='capturing'):
        export_archive(str(tmp_path), 'run-4')
    with pytest.raises(FileExistsError):             # a restart can never reuse / resume the file
        RunRecorder(str(tmp_path), 'run-4', TOPICS)


def test_samples_after_finish_are_ignored(tmp_path):
    rec = RunRecorder(str(tmp_path), 'run-5', TOPICS)
    rec.record(TOPICS[0], b'1')
    rec.finish()
    rec.record(TOPICS[0], b'2')
    rec.join()
    assert export_archive(str(tmp_path), 'run-5')['expected_count'] == 1


def test_cli_writes_archive(tmp_path, capsys):
    rec = RunRecorder(str(tmp_path), 'run-6', TOPICS, snapshot={'k': 1})
    rec.record(TOPICS[1], b'zz')
    rec.finish()
    rec.join()
    assert cli.main(['run-6', '--dir', str(tmp_path)]) == 0
    assert json.load(open(tmp_path / 'run-6.archive.json'))['expected_count'] == 1
    assert json.load(open(tmp_path / 'run-6.params.json')) == {'k': 1}
    assert cli.main(['nope', '--dir', str(tmp_path)]) == 1


@pytest.mark.parametrize('rid,ok', [('abc-1_2.x', True), ('', False), ('../x', False), ('a/b', False), ('a..b', False),
                                    ('a' * 129, False), ('-lead', False)])
def test_run_id_validation(rid, ok):
    assert valid_run_id(rid) is ok
