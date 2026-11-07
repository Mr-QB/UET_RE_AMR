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

"""Clean lidar artefacts out of a saved trinary occupancy map (.pgm) in place.

Run by save_map.launch.py after map_saver_cli; also usable by hand: clean_map.py <map.pgm>
"""

import sys

import cv2
import numpy as np

# map_saver_cli trinary pixel values.
_PGM_OCCUPIED, _PGM_UNKNOWN, _PGM_FREE = 0, 205, 254


def clean_pgm(path, min_occupied_px=12, min_free_px=25, free_ray_width_px=3):
    """Remove incomplete scan artefacts from a saved trinary map in place; returns the number of pixels changed.

    Occupied blobs smaller than min_occupied_px (stray hits, ragged reflections beside walls) become
    free if they touch free space, else unknown. Free structures thinner than free_ray_width_px (the
    single beams that poke into unknown space) and free blobs smaller than min_free_px become unknown.
    """
    image = cv2.imread(path, cv2.IMREAD_UNCHANGED)
    if image is None or image.ndim != 2:
        raise RuntimeError(f'Could not read {path} as a grayscale image')
    cleaned = image.copy()
    kernel = np.ones((3, 3), np.uint8)

    def small_blobs(mask, limit):
        count, labels, stats, _ = cv2.connectedComponentsWithStats(mask.astype(np.uint8), connectivity=8)
        return np.isin(labels, [i for i in range(1, count) if stats[i, cv2.CC_STAT_AREA] < limit])

    free = image == _PGM_FREE
    speckle = small_blobs(image == _PGM_OCCUPIED, min_occupied_px)
    beside_free = speckle & cv2.dilate(free.astype(np.uint8), kernel).astype(bool)
    cleaned[speckle] = _PGM_UNKNOWN
    cleaned[beside_free] = _PGM_FREE

    opened = cv2.morphologyEx(free.astype(np.uint8), cv2.MORPH_OPEN,
                              cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (free_ray_width_px,) * 2)).astype(bool)
    cleaned[free & ~opened] = _PGM_UNKNOWN
    cleaned[small_blobs(cleaned == _PGM_FREE, min_free_px)] = _PGM_UNKNOWN

    changed = int(np.count_nonzero(cleaned != image))
    if changed:
        cv2.imwrite(path, cleaned)
    return changed


def main():
    if len(sys.argv) != 2:
        print('usage: clean_map.py <map.pgm>', file=sys.stderr)
        return 2
    try:
        changed = clean_pgm(sys.argv[1])
        print(f'Cleaned {changed} artefact pixels from {sys.argv[1]}')
    except (RuntimeError, cv2.error, OSError) as exc:
        # The map itself is already saved; a failed cleanup must not fail the save.
        print(f'WARNING: map saved, but cleanup failed: {exc}', file=sys.stderr)
    return 0


if __name__ == '__main__':
    sys.exit(main())
