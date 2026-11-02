#!/usr/bin/env python3
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
Robot-side consumer of the dashboard's saved-map zones (schema version 1).

/amr/apply_zones takes the exported JSON, checks it against the active map (fingerprint, frame, yaw == 0), and builds a
keepout mask (no_go zones) and an absolute speed mask (slow zones). It persists the zones next to the map, then
publishes both masks with their CostmapFilterInfo on transient-local topics for the Nav2 KeepoutFilter / SpeedFilter.
Labels are semantic only and never restrict. The previous revision stays active when anything fails.

Masks are published only for maps with origin yaw 0: Nav2 Humble's StaticLayer and costmap filters ignore the yaw.
"""
import array
import hashlib
import json
import math
import os
import struct
import threading

import numpy as np
import yaml

SCHEMA_VERSION = 1
MAX_ZONES = 200
MAX_VERTICES = 128
MAX_NAME = 100
ZONE_TYPES = ('label', 'no_go', 'slow')
EPS = 1e-9


class ZoneError(ValueError):
    """The request cannot be applied; str() is returned to the caller."""


# ---------------------------------------------------------------------------------------------------------------
# Active map
# ---------------------------------------------------------------------------------------------------------------

def _pgm_size(data):
    """Width and height from a binary (P5) or ASCII (P2) PGM header."""
    tokens, pos = [], 0
    while len(tokens) < 3 and pos < len(data):
        if data[pos:pos + 1] == b'#':
            while pos < len(data) and data[pos:pos + 1] not in (b'\n', b'\r'):
                pos += 1
        elif data[pos:pos + 1].isspace():
            pos += 1
        else:
            start = pos
            while pos < len(data) and not data[pos:pos + 1].isspace():
                pos += 1
            tokens.append(data[start:pos])
    if len(tokens) < 3 or tokens[0] not in (b'P5', b'P2'):
        raise ZoneError('active map image is not a PGM')
    return int(tokens[1]), int(tokens[2])


def read_active_map(map_yaml):
    """Fingerprint (same recipe as the dashboard) and geometry of the map the robot is running on."""
    if not map_yaml:
        raise ZoneError('active map is unknown')
    try:
        with open(map_yaml, 'rb') as f:
            yaml_bytes = f.read()
        info = yaml.safe_load(yaml_bytes)
        # Like /amr/get_map: the image sits next to the YAML, only its file name counts.
        image_name = os.path.basename(str(info['image']))
        with open(os.path.join(os.path.dirname(map_yaml), image_name), 'rb') as f:
            image = f.read()
        origin = [float(v) for v in info['origin']]
        resolution = float(info['resolution'])
        width, height = _pgm_size(image)
    except (OSError, KeyError, TypeError, ValueError, yaml.YAMLError) as exc:
        raise ZoneError(f'cannot read the active map {map_yaml}: {exc}') from exc
    fingerprint = hashlib.sha256(
        struct.pack('>Q', len(yaml_bytes)) + yaml_bytes + image).hexdigest()
    return {'fingerprint': fingerprint, 'width': width, 'height': height, 'resolution': resolution,
            'origin_x': origin[0], 'origin_y': origin[1], 'origin_yaw': origin[2]}


# ---------------------------------------------------------------------------------------------------------------
# Validation
# ---------------------------------------------------------------------------------------------------------------

def _cross(o, a, b):
    return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])


def _on_segment(p, a, b):
    return min(a[0], b[0]) <= p[0] <= max(a[0], b[0]) and min(a[1], b[1]) <= p[1] <= max(a[1], b[1])


def _segments_touch(a, b, c, d):
    d1, d2, d3, d4 = _cross(a, b, c), _cross(a, b, d), _cross(c, d, a), _cross(c, d, b)
    if ((d1 > 0) != (d2 > 0)) and d1 != 0 and d2 != 0 and ((d3 > 0) != (d4 > 0)) and d3 != 0 and d4 != 0:
        return True
    return ((d1 == 0 and _on_segment(c, a, b)) or (d2 == 0 and _on_segment(d, a, b))
            or (d3 == 0 and _on_segment(a, c, d)) or (d4 == 0 and _on_segment(b, c, d)))


def check_polygon(points, name):
    """Simple polygon: no repeated vertices, zero area, self-touching, or overlapping edges."""
    n = len(points)
    if len(set(points)) != n:
        raise ZoneError(f'zone {name!r} has repeated vertices')
    for i in range(n):
        a, b = points[i], points[(i + 1) % n]
        c = points[(i + 2) % n]
        if _cross(a, b, c) == 0 and (a[0] - b[0]) * (c[0] - b[0]) + (a[1] - b[1]) * (c[1] - b[1]) > 0:
            raise ZoneError(f'zone {name!r} has overlapping edges')
        for j in range(i + 2, n):
            if i == 0 and j == n - 1:
                continue
            if _segments_touch(a, b, points[j], points[(j + 1) % n]):
                raise ZoneError(f'zone {name!r} self-intersects')
    area2 = sum(points[i][0] * points[(i + 1) % n][1] - points[(i + 1) % n][0] * points[i][1] for i in range(n))
    if area2 == 0:
        raise ZoneError(f'zone {name!r} has zero area')


def parse_export(text, active):
    """Validate the export JSON against the active map; returns the zone list as dicts with 'points' tuples."""
    try:
        doc = json.loads(text)
    except (TypeError, ValueError) as exc:
        raise ZoneError(f'zones_json is not valid JSON: {exc}') from exc
    try:
        if doc['schema_version'] != SCHEMA_VERSION:
            raise ZoneError(f'unsupported schema_version {doc["schema_version"]!r}')
        meta, revision, raw_zones = doc['map'], doc['revision'], doc['zones']
        if (isinstance(revision, bool) or not isinstance(revision, int) or revision < 1
                or revision > 0xFFFFFFFF):
            raise ZoneError('revision must be a positive integer')
        if meta['frame'] != 'map':
            raise ZoneError(f'unsupported frame {meta["frame"]!r}; expected "map"')
        if meta['fingerprint'] != active['fingerprint']:
            raise ZoneError('zones were drawn on a different map version than the active map '
                            '(fingerprint mismatch)')
        if abs(active['origin_yaw']) > EPS:
            raise ZoneError('active map origin yaw is not 0; Nav2 Humble costmap filters ignore yaw, so zones '
                            'cannot be applied until the map is normalized')
        if not isinstance(raw_zones, list) or len(raw_zones) > MAX_ZONES:
            raise ZoneError(f'zones must be a list of at most {MAX_ZONES}')
        zones, ids = [], set()
        extent_u = active['width'] * active['resolution']
        extent_v = active['height'] * active['resolution']
        for raw in raw_zones:
            zid, zname, ztype = raw['id'], raw['name'], raw['type']
            if not isinstance(zid, str) or not zid or zid in ids:
                raise ZoneError('zone ids must be unique non-empty strings')
            ids.add(zid)
            if not isinstance(zname, str) or not zname.strip() or len(zname) > MAX_NAME:
                raise ZoneError(f'zone {zid!r} has an invalid name')
            if ztype not in ZONE_TYPES:
                raise ZoneError(f'zone {zname!r} has unknown type {ztype!r}')
            limit = raw.get('speed_limit_mps')
            if ztype == 'slow':
                if (isinstance(limit, bool) or not isinstance(limit, (int, float))
                        or not math.isfinite(limit) or limit <= 0):
                    raise ZoneError(f'slow zone {zname!r} needs a finite speed_limit_mps > 0')
            elif limit is not None:
                raise ZoneError(f'zone {zname!r} must have speed_limit_mps null')
            verts = raw['vertices']
            if not isinstance(verts, list) or not 3 <= len(verts) <= MAX_VERTICES:
                raise ZoneError(f'zone {zname!r} needs 3 to {MAX_VERTICES} vertices')
            points = []
            for v in verts:
                x, y = v['x'], v['y']
                if any(isinstance(c, bool) or not isinstance(c, (int, float)) or not math.isfinite(c)
                       for c in (x, y)):
                    raise ZoneError(f'zone {zname!r} has a non-finite vertex')
                u, w = x - active['origin_x'], y - active['origin_y']
                if not -EPS <= u <= extent_u + EPS or not -EPS <= w <= extent_v + EPS:
                    raise ZoneError(f'zone {zname!r} extends outside the map')
                points.append((float(u), float(w)))
            check_polygon(points, zname)
            zones.append({'id': zid, 'name': zname, 'type': ztype, 'limit': limit, 'points': points})
    except ZoneError:
        raise
    except (KeyError, TypeError, AttributeError) as exc:
        raise ZoneError(f'malformed zones export: {exc!r}') from exc
    return revision, zones


# ---------------------------------------------------------------------------------------------------------------
# Rasterization (map-local meters; row 0 is the bottom row, matching OccupancyGrid layout)
# ---------------------------------------------------------------------------------------------------------------

def rasterize(points, width, height, res):
    """
    Conservative raster of one polygon: every cell whose centre is inside or that its boundary touches.
    Returns (row0, col0, bool array) covering the polygon's bounding box.
    """
    pts = np.asarray(points, dtype=float)
    col0 = max(int(math.floor(pts[:, 0].min() / res - EPS)), 0)
    col1 = min(int(math.floor(pts[:, 0].max() / res + EPS)), width - 1)
    row0 = max(int(math.floor(pts[:, 1].min() / res - EPS)), 0)
    row1 = min(int(math.floor(pts[:, 1].max() / res + EPS)), height - 1)
    rows, cols = row1 - row0 + 1, col1 - col0 + 1
    diff = np.zeros((rows, cols + 1), dtype=np.int32)

    def add_spans(r, c_lo, c_hi):
        c_lo, c_hi = np.clip(c_lo, 0, cols), np.clip(c_hi + 1, 0, cols)
        keep = c_hi > c_lo
        np.add.at(diff, (r[keep], c_lo[keep]), 1)
        np.add.at(diff, (r[keep], c_hi[keep]), -1)

    # Interior: cell centres by even-odd scanline fill.
    cy = (np.arange(row0, row1 + 1) + 0.5) * res
    x1, y1 = pts[:, 0], pts[:, 1]
    x2, y2 = np.roll(x1, -1), np.roll(y1, -1)
    spans = (y1[None, :] <= cy[:, None]) != (y2[None, :] <= cy[:, None])
    dy = np.where(y2 == y1, 1.0, y2 - y1)
    xs = x1[None, :] + (cy[:, None] - y1[None, :]) * (x2 - x1)[None, :] / dy[None, :]
    xs = np.sort(np.where(spans, xs, np.inf), axis=1)
    for k in range(0, xs.shape[1] - 1, 2):
        a, b = xs[:, k], xs[:, k + 1]
        ok = np.isfinite(a) & np.isfinite(b)
        idx = np.nonzero(ok)[0]
        c_lo = np.ceil(a[ok] / res - 0.5 - EPS).astype(int) - col0
        c_hi = np.floor(b[ok] / res - 0.5 + EPS).astype(int) - col0
        add_spans(idx, c_lo, c_hi)

    # Boundary: per edge and per row band, the columns the clipped segment passes through.
    for (ax, ay), (bx, by) in zip(pts, np.roll(pts, -1, axis=0)):
        j_lo = max(int(math.floor(min(ay, by) / res - EPS)), row0)
        j_hi = min(int(math.floor(max(ay, by) / res + EPS)), row1)
        if j_hi < j_lo:
            continue
        j = np.arange(j_lo, j_hi + 1)
        if by == ay:
            ua = np.full(j.shape, min(ax, bx))
            ub = np.full(j.shape, max(ax, bx))
        else:
            slope = (bx - ax) / (by - ay)
            t0 = np.clip(j * res, min(ay, by), max(ay, by))
            t1 = np.clip((j + 1) * res, min(ay, by), max(ay, by))
            xa, xb = ax + (t0 - ay) * slope, ax + (t1 - ay) * slope
            ua, ub = np.minimum(xa, xb), np.maximum(xa, xb)
        c_lo = np.floor(ua / res - EPS).astype(int) - col0
        c_hi = np.floor(ub / res + EPS).astype(int) - col0
        add_spans(j - row0, c_lo, c_hi)

    return row0, col0, np.cumsum(diff, axis=1)[:, :cols] > 0


def build_masks(zones, width, height, res):
    """Keepout mask (100 = no-go) and speed mask (0 = unrestricted); returns (keepout, speed, speed_multiplier)."""
    keepout = np.zeros((height, width), dtype=np.int8)
    speed = np.zeros((height, width), dtype=np.int8)
    slow = [z for z in zones if z['type'] == 'slow']
    maximum = max((z['limit'] for z in slow), default=0.0)
    multiplier = maximum / 100.0 if slow else 1.0
    for zone in zones:
        if zone['type'] == 'label':
            continue
        value = 0
        if zone['type'] == 'slow':
            value = int(math.floor(zone['limit'] / multiplier + 1e-9))
            if value < 1:
                raise ZoneError(
                    f'slow zone {zone["name"]!r} ({zone["limit"]} m/s) is below 1% of the fastest slow limit '
                    f'({maximum} m/s) and cannot be encoded; it would silently become unrestricted')
        r0, c0, hit = rasterize(zone['points'], width, height, res)
        if zone['type'] == 'no_go':
            keepout[r0:r0 + hit.shape[0], c0:c0 + hit.shape[1]][hit] = 100
        else:
            region = speed[r0:r0 + hit.shape[0], c0:c0 + hit.shape[1]]
            region[hit & ((region == 0) | (region > value))] = value
    return keepout, speed, multiplier


# ---------------------------------------------------------------------------------------------------------------
# Node
# ---------------------------------------------------------------------------------------------------------------

def _write_atomic(path, text):
    tmp = path + '.tmp'
    with open(tmp, 'w', encoding='utf-8') as f:
        f.write(text)
        f.flush()
        os.fsync(f.fileno())
    os.replace(tmp, path)


def main(args=None):
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from nav_msgs.msg import OccupancyGrid
    from nav2_msgs.msg import CostmapFilterInfo
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from uet_amr_msgs.srv import ApplyZones

    class ZonesManager(Node):
        def __init__(self):
            super().__init__('zones_manager')
            self.declare_parameter('map', '')
            self.declare_parameter('zones_file', '')
            self._map_yaml = str(self.get_parameter('map').value)
            zones_file = str(self.get_parameter('zones_file').value)
            if not zones_file and self._map_yaml:
                zones_file = os.path.join(os.path.dirname(self._map_yaml), 'zones.json')
            self._zones_file = zones_file
            self._lock = threading.Lock()
            self._fingerprint, self._revision = '', 0

            qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
            self._pubs = {
                'keepout': (self.create_publisher(OccupancyGrid, '/keepout_filter_mask', qos),
                            self.create_publisher(CostmapFilterInfo, '/keepout_costmap_filter_info', qos)),
                'speed': (self.create_publisher(OccupancyGrid, '/speed_filter_mask', qos),
                          self.create_publisher(CostmapFilterInfo, '/speed_costmap_filter_info', qos)),
            }
            self.create_service(ApplyZones, '/amr/apply_zones', self._apply_callback)
            self._restore()
            self.get_logger().info('Service ready: /amr/apply_zones (uet_amr_msgs/srv/ApplyZones)')

        def _restore(self):
            """Reinstall the persisted zones; a missing, stale, or invalid file leaves the robot unrestricted."""
            if not self._zones_file or not os.path.isfile(self._zones_file):
                return
            try:
                with open(self._zones_file, 'r', encoding='utf-8') as f:
                    self._install(f.read(), persist=False)
            except (OSError, ZoneError) as exc:
                self.get_logger().warning(f'Not restoring {self._zones_file}: {exc}')

        def _install(self, text, persist):
            active = read_active_map(self._map_yaml)
            revision, zones = parse_export(text, active)
            if active['fingerprint'] == self._fingerprint and revision < self._revision:
                raise ZoneError(f'revision {revision} is older than the applied revision {self._revision}')
            keepout, speed, multiplier = build_masks(
                zones, active['width'], active['height'], active['resolution'])
            if persist:
                if not self._zones_file:
                    raise ZoneError('no zones_file; cannot persist zones')
                try:
                    _write_atomic(self._zones_file, text)
                except OSError as exc:
                    raise ZoneError(f'cannot persist zones: {exc}') from exc
            stamp = self.get_clock().now().to_msg()
            self._publish('keepout', keepout, 0, 1.0, active, stamp)
            self._publish('speed', speed, 2, multiplier, active, stamp)
            self._fingerprint, self._revision = active['fingerprint'], revision
            counts = (int((keepout == 100).sum()), int((speed > 0).sum()))
            self.get_logger().info(
                f'Applied zones revision {revision}: {len(zones)} zone(s), {counts[0]} keepout cell(s), '
                f'{counts[1]} slow cell(s), speed multiplier {multiplier:.4f}')

        def _publish(self, kind, data, filter_type, multiplier, active, stamp):
            grid = OccupancyGrid()
            grid.header.frame_id = 'map'
            grid.header.stamp = stamp
            grid.info.map_load_time = stamp
            grid.info.resolution = active['resolution']
            grid.info.width, grid.info.height = active['width'], active['height']
            grid.info.origin.position.x = active['origin_x']
            grid.info.origin.position.y = active['origin_y']
            grid.info.origin.orientation.w = 1.0
            grid.data = array.array('b', data.tobytes())
            info = CostmapFilterInfo()
            info.header.frame_id = 'map'
            info.header.stamp = stamp
            info.type = filter_type
            info.filter_mask_topic = f'/{kind}_filter_mask'
            info.base = 0.0
            info.multiplier = float(multiplier)
            mask_pub, info_pub = self._pubs[kind]
            # Mask first so a filter that reacts to the info already has data to subscribe to.
            mask_pub.publish(grid)
            info_pub.publish(info)

        def _apply_callback(self, request, response):
            with self._lock:
                try:
                    self._install(request.zones_json, persist=True)
                    response.success, response.message = True, f'Applied revision {self._revision}'
                except ZoneError as exc:
                    self.get_logger().warning(f'Rejected zones: {exc}')
                    response.success, response.message = False, str(exc)
                except Exception as exc:  # keep the previous revision and keep serving
                    self.get_logger().error(f'Applying zones failed: {exc!r}')
                    response.success, response.message = False, f'internal error: {exc!r}'
                response.applied_fingerprint = self._fingerprint
                response.applied_revision = self._revision
            return response

    rclpy.init(args=args)
    node = ZonesManager()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
