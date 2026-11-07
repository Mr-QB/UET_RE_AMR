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

import importlib.util
import json
import os

import numpy as np
import pytest

_spec = importlib.util.spec_from_file_location(
    'zones_manager', os.path.join(os.path.dirname(__file__), '..', 'scripts', 'zones_manager.py'))
zm = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(zm)

W, H, RES = 20, 10, 0.5


def make_map(tmp_path, yaw=0.0):
    (tmp_path / 'm.pgm').write_bytes(b'P5\n# c\n20 10\n255\n' + bytes([254]) * (W * H))
    (tmp_path / 'm.yaml').write_text(
        f'image: m.pgm\nresolution: {RES}\norigin: [-2.0, -1.0, {yaw}]\nnegate: 0\n')
    return str(tmp_path / 'm.yaml')


def rect(x0, y0, x1, y1):
    return [{'x': x0, 'y': y0}, {'x': x1, 'y': y0}, {'x': x1, 'y': y1}, {'x': x0, 'y': y1}]


def zone(zid, ztype, verts, limit=None):
    return {'id': zid, 'name': zid, 'type': ztype, 'vertices': verts, 'speed_limit_mps': limit}


def export(active, zones, revision=1, **meta):
    return json.dumps({
        'schema_version': 1, 'revision': revision, 'zones': zones,
        'map': {'frame': 'map', 'fingerprint': active['fingerprint'], **meta}})


def test_fingerprint_changes_with_yaml(tmp_path):
    a = zm.read_active_map(make_map(tmp_path))
    assert (a['width'], a['height'], a['origin_x']) == (W, H, -2.0)
    (tmp_path / 'm.yaml').write_text((tmp_path / 'm.yaml').read_text() + '# note\n')
    assert zm.read_active_map(str(tmp_path / 'm.yaml'))['fingerprint'] != a['fingerprint']


def test_rect_raster_is_conservative():
    # Local box u in [1.0, 2.0], v in [0.5, 1.0]: cols 1..4, rows 0..2.
    r0, c0, hit = zm.rasterize([(1.0, 0.5), (2.0, 0.5), (2.0, 1.0), (1.0, 1.0)], W, H, RES)
    full = np.zeros((H, W), bool)
    full[r0:r0 + hit.shape[0], c0:c0 + hit.shape[1]] = hit
    assert full[0:3, 1:5].all() and full.sum() == 12


def test_tiny_polygon_inside_one_cell_is_marked():
    _, _, hit = zm.rasterize([(1.1, 1.1), (1.2, 1.1), (1.15, 1.2)], W, H, RES)
    assert hit.sum() == 1


def test_concave_notch_is_not_filled_but_boundary_cells_are():
    pts = [(0, 0), (6, 0), (6, 4), (4, 4), (4, 1), (2, 1), (2, 4), (0, 4)]  # U shape, notch u in [2,4]
    r0, c0, hit = zm.rasterize(pts, 20, 10, 0.5)
    assert r0 == 0 and c0 == 0
    assert not hit[3, 5] and not hit[3, 6]    # notch interior (u 2.5..3.5, v 1.5..2) stays free
    assert hit[3, 4] and hit[3, 7] and hit[2, 5]  # cells touching the notch boundary are marked


def test_masks_overlap_policy(tmp_path):
    active = zm.read_active_map(make_map(tmp_path))
    zones = [
        zm.parse_export(export(active, [
            zone('a', 'slow', rect(-2, -1, 2, 2), 1.0),
            zone('b', 'slow', rect(0, 0, 3, 3), 0.25),
            zone('c', 'no_go', rect(0, 0, 1, 1)),
            zone('l', 'label', rect(-2, -1, 3, 3)),
        ]), active)[1]][0]
    keepout, speed, mult = zm.build_masks(zones, W, H, RES)
    assert mult == pytest.approx(0.01)
    assert keepout.max() == 100 and keepout.sum() == 100 * (keepout == 100).sum()
    # u = x + 2, v = y + 1: point (1, 1) is local (3, 2) -> col 6, row 4, inside both slow zones: lowest wins.
    assert speed[4, 6] == 25
    assert speed[0, 0] == 100 and speed[9, 19] == 0
    assert (speed[keepout == 100] > 0).all()        # slow still encoded under no-go; filters are independent


def test_unencodable_slow_limit_rejected(tmp_path):
    active = zm.read_active_map(make_map(tmp_path))
    _, zones = zm.parse_export(export(active, [
        zone('a', 'slow', rect(-2, -1, 0, 0), 2.0), zone('b', 'slow', rect(1, 1, 2, 2), 0.01)]), active)
    with pytest.raises(zm.ZoneError, match='cannot be encoded'):
        zm.build_masks(zones, W, H, RES)


@pytest.mark.parametrize('mutate, message', [
    (lambda d: d.update(schema_version=2), 'schema_version'),
    (lambda d: d['map'].update(fingerprint='x'), 'fingerprint'),
    (lambda d: d['map'].update(frame='odom'), 'frame'),
    (lambda d: d.update(revision=0), 'revision'),
    (lambda d: d['zones'].append(d['zones'][0]), 'unique'),
    (lambda d: d['zones'][0].update(vertices=rect(-3, 0, 0, 1)), 'outside'),
    (lambda d: d['zones'][0].update(vertices=[{'x': 0, 'y': 0}, {'x': 1, 'y': 1}, {'x': 1, 'y': 0}, {'x': 0, 'y': 1}]),
     'self-intersects'),
    (lambda d: d['zones'][0].update(type='slow'), 'speed_limit_mps'),
    (lambda d: d['zones'][0].update(vertices=rect(0, 0, 1, 1)[:2]), 'vertices'),
    (lambda d: d['zones'][0].update(vertices=rect(0, 0, 1, 1) + [{'x': 0, 'y': 0}]), 'repeated'),
])
def test_invalid_exports(tmp_path, mutate, message):
    active = zm.read_active_map(make_map(tmp_path))
    doc = json.loads(export(active, [zone('a', 'no_go', rect(0, 0, 1, 1))]))
    mutate(doc)
    with pytest.raises(zm.ZoneError, match=message):
        zm.parse_export(json.dumps(doc), active)


def test_nonzero_yaw_rejected(tmp_path):
    active = zm.read_active_map(make_map(tmp_path, yaw=0.3))
    with pytest.raises(zm.ZoneError, match='yaw'):
        zm.parse_export(export(active, []), active)
