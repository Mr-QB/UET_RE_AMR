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
Export a finished run's robot-side capture as the dashboard JSON archive.

  ros2 run uet_amr_evaluation export_motion_archive <run_id> [--dir DIR] [--out FILE]

Upload the file via the dashboard's run inspector, or POST it to /api/experiments/<run_id>/robot-archive.
Re-exporting a run gives a byte-identical archive (same archive_id), so re-uploads are idempotent.
The controller / filter parameter snapshot is written next to it as <run_id>.params.json.
"""
import argparse
import json
import os
import sys

from uet_amr_evaluation.motion.recorder import ArchiveError, export_archive, load_snapshot

DEFAULT_DIR = os.path.expanduser('~/amr_motion_tests')


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('run_id')
    p.add_argument('--dir', default=DEFAULT_DIR)
    p.add_argument('--out')
    args = p.parse_args(argv)
    try:
        archive = export_archive(args.dir, args.run_id)
        snapshot = load_snapshot(args.dir, args.run_id)
    except (ArchiveError, OSError) as exc:
        print(f'error: {exc}', file=sys.stderr)
        return 1
    out = args.out or os.path.join(args.dir, args.run_id + '.archive.json')
    with open(out, 'w', encoding='utf-8') as f:
        json.dump(archive, f, separators=(',', ':'))
    with open(os.path.join(os.path.dirname(out) or '.', args.run_id + '.params.json'), 'w', encoding='utf-8') as f:
        json.dump(snapshot, f, indent=2)
    print(f'{out}: {archive["expected_count"]} packets, archive_id {archive["archive_id"]}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
