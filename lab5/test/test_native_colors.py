"""Native starter detector regressions for the 2026-11-12 color profile."""

import copy

import cv2
import numpy as np
import pytest

from ece470labs.blob_search import NATIVE_VISION_DEFAULTS, blob_search


def bgr(hue, saturation=190, value=190):
    """Build a BGR sample with known OpenCV HSV values."""
    return tuple(map(int, cv2.cvtColor(np.uint8([[[hue, saturation, value]]]), cv2.COLOR_HSV2BGR)[0, 0]))


def test_native_green_yellow_masks_are_distinct_and_reject_distractors():
    image = np.full((480, 640, 3), 170, np.uint8)
    centers = {'green': [(250, 220), (350, 260)], 'yellow': [(300, 220), (400, 260)]}
    for color, hue in [('green', 57), ('yellow', 20)]:
        for x, y in centers[color]:
            cv2.rectangle(image, (x-10, y-10), (x+10, y+10), bgr(hue), -1)
    cv2.rectangle(image, (200, 50), (220, 70), bgr(57), -1)  # Outside table ROI.
    cv2.rectangle(image, (150, 200), (175, 225), bgr(110), -1)  # Blue object.
    cv2.rectangle(image, (450, 200), (500, 250), bgr(20), -1)  # Oversized object.
    cv2.rectangle(image, (200, 300), (202, 302), bgr(57), -1)  # Tiny noise.
    for color in ['green', 'yellow']:
        _, _, masks, blocks = blob_search(image, color, show=False, return_details=True)
        assert len(blocks) == 2
        np.testing.assert_allclose(sorted((d.col, d.row) for d in blocks), centers[color], atol=.2)
        assert all(d.world_xy is None for d in blocks)
        other = centers['yellow' if color == 'green' else 'green'][0]
        assert masks[color][other[1], other[0]] == 0


@pytest.mark.parametrize('problem', ['unknown_color', 'hsv', 'roi', 'area'])
def test_native_invalid_settings_fail_clearly(problem):
    vision = copy.deepcopy(NATIVE_VISION_DEFAULTS)
    color = 'green'
    if problem == 'unknown_color':
        color = 'purple'
    elif problem == 'hsv':
        vision['hsv']['green'][0][0] = 100
    elif problem == 'roi':
        vision['roi'] = [0, 0, 700, 480]
    else:
        vision['area_px'] = [1200, 180]
    with pytest.raises(ValueError):
        blob_search(np.zeros((480, 640, 3), np.uint8), color, show=False,
                    return_details=True, vision=vision)
