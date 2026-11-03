import copy
import json
import math
from pathlib import Path
import sys
import subprocess
import tempfile
import unittest
import cv2
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from core import detect, project, rank_targets, field_position, validate_config
from run import DEFAULT_CONFIG
from export_limelight import TAIL


def config(calibrated=True):
    c = json.loads(DEFAULT_CONFIG.read_text())
    if calibrated:
        c['calibration'] = {'homography': [[0, -60, 70], [-60, 0, 30], [0, 0, 1]],
                            'polygon': [[.05, .05], [.95, .05], [.95, .95], [.05, .95]], 'aspect': 4/3}
    return validate_config(c)


def scene():
    image = np.zeros((480, 640, 3), np.uint8)
    cv2.circle(image, (200, 300), 24, (0, 255, 255), -1)
    cv2.circle(image, (260, 300), 24, (0, 0, 255), -1)
    cv2.circle(image, (510, 120), 24, (255, 0, 0), -1)
    return image


class DetectionTest(unittest.TestCase):
    def test_colors_and_blank(self):
        r, _ = detect(scene(), config(), True)
        self.assertEqual(r['counts'], {'POLLEN': 1, 'RED_NECTAR': 1, 'BLUE_NECTAR': 1})
        self.assertEqual(r['best']['counts']['POLLEN'], 1)
        self.assertEqual(r['best']['counts']['RED_NECTAR'], 1)
        empty, _ = detect(np.zeros((480, 640, 3), np.uint8), config(), True)
        self.assertIsNone(empty['best'])
        self.assertEqual(len(empty['blobs']), 0)

    def test_no_metric_without_calibration_or_intake(self):
        for c, ready in [(config(False), True), (config(), False)]:
            r, _ = detect(scene(), c, ready)
            self.assertEqual(len(r['blobs']), 3)
            self.assertIsNone(r['best'])
            self.assertTrue(all(b['position'] is None for b in r['blobs']))

    def test_red_hue_wrap(self):
        image = np.zeros((480, 640, 3), np.uint8)
        hsv = np.zeros_like(image)
        cv2.circle(hsv, (200, 200), 25, (175, 255, 255), -1)
        r, _ = detect(cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR), config())
        self.assertEqual(r['counts']['RED_NECTAR'], 1)

    def test_clipped_roi_and_noise(self):
        image = np.zeros((480, 640, 3), np.uint8)
        cv2.circle(image, (0, 300), 30, (0, 255, 255), -1)
        image[100, 100] = (0, 255, 255)
        r, _ = detect(image, config(), True)
        self.assertEqual(len(r['blobs']), 1)
        self.assertTrue(r['blobs'][0]['clipped'])
        self.assertIsNone(r['best'])
        c = config(); c['roi'] = [.5, 0, 1, 1]
        r, _ = detect(scene(), c, True)
        self.assertEqual(r['counts'], {'POLLEN': 0, 'RED_NECTAR': 0, 'BLUE_NECTAR': 1})

    def test_limit_and_merged_blobs(self):
        c = config(); c['max_blobs_per_color'] = 1
        image = scene(); cv2.circle(image, (100, 100), 25, (0, 255, 255), -1)
        r, _ = detect(image, c, True)
        self.assertTrue(r['truncated'])
        self.assertEqual(r['counts']['POLLEN'], 1)
        image = np.zeros((480, 640, 3), np.uint8)
        cv2.circle(image, (200, 300), 25, (0,255,255), -1)
        cv2.circle(image, (230, 300), 25, (0,255,255), -1)
        r, _ = detect(image, config(), True)
        self.assertEqual(r['counts']['POLLEN'], 1)  # explicitly a blob, not two counted balls

    def test_positions_bounds_and_field_transform(self):
        c = config()
        self.assertEqual(project(.5, .5, c, 4/3), [40, 0])
        self.assertIsNone(project(0, .5, c, 4/3))
        self.assertIsNone(project(.5, .5, c, 16/9))
        c['max_range_inches'] = 30
        self.assertIsNone(project(.5, .5, c, 4/3))
        point = field_position([10, 5], [100, 200, math.pi/2])
        self.assertAlmostEqual(point[0], 95)
        self.assertAlmostEqual(point[1], 210)

    def test_density_alliance_and_mode(self):
        blobs = [{'kind': k, 'position': p} for k, p in [
            ('POLLEN', [10, 0]), ('POLLEN', [35, 0]), ('POLLEN', [36, 0]),
            ('RED_NECTAR', [60, 0]), ('RED_NECTAR', [61, 0]), ('RED_NECTAR', [62, 0]),
            ('BLUE_NECTAR', [5, 0])]]
        c = config(); c['cluster_radius_inches'] = 4; c['distance_penalty_per_inch'] = .01
        self.assertEqual(rank_targets(blobs, c, 'RED', 'MIXED')[0]['position'], [60, 0])
        self.assertEqual(rank_targets(blobs, c, 'RED', 'POLLEN_ONLY')[0]['position'], [35, 0])
        self.assertEqual(rank_targets(blobs, c, 'BLUE', 'MIXED')[0]['position'], [35, 0])
        self.assertFalse(any(t['blob_index'] == 5 for t in rank_targets(blobs, c, 'BLUE')))

    def test_invalid_config(self):
        for key, value in [('morph_kernel', 2), ('max_range_inches', float('nan')), ('roi', [1, 0, 0, 1])]:
            c = config(); c[key] = value
            with self.assertRaises(ValueError): validate_config(c)
        c = config(); c['calibration']['homography'] = [[0]*3]*3
        with self.assertRaises(ValueError): validate_config(c)

    def test_limelight_protocol(self):
        namespace = {'np': np, 'cv2': cv2, 'detect': detect, 'CONFIG': config()}
        exec(TAIL, namespace)
        pipeline = namespace['runPipeline']
        _, _, out = pipeline(scene(), [42, 1, 0, 0])
        self.assertEqual(len(out), 8)
        self.assertEqual(out[0], 42)
        self.assertEqual(out[4:7], [1, 1, 1])
        _, _, disabled = pipeline(scene(), [43, 0, 0, 0])
        self.assertEqual(disabled[0], -43)
        self.assertEqual(disabled[1:4], [0, 0, 0])
        self.assertNotEqual(out[7], disabled[7])
        _, _, blank = pipeline(np.zeros((480, 640, 3), np.uint8), [44, 1, 0, 0])
        self.assertEqual(blank[0], -44)
        self.assertEqual(blank[4:7], [0, 0, 0])

    def test_headless_saved_frame_is_not_blocked_by_servo_timer(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / 'config.json').write_text(json.dumps(config()))
            cv2.imwrite(str(root / 'frame.png'), scene())
            subprocess.run([sys.executable, str(Path(__file__).resolve().parents[1] / 'run.py'),
                            '--image', str(root / 'frame.png'), '--config', str(root / 'config.json'),
                            '--intake', '--headless', '--jsonl', str(root / 'results.jsonl'),
                            '--output', str(root / 'output')], check=True, capture_output=True)
            rows = (root / 'results.jsonl').read_text().splitlines()
            self.assertEqual(len(rows), 1)
            self.assertIsNotNone(json.loads(rows[0])['best'])


if __name__ == '__main__': unittest.main()
