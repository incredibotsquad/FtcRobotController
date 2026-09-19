"""BIOBuzz OpenCV reference implementation; also bundled verbatim for Limelight.
Coordinates: inches, +forward and +left from the robot's chosen origin.
Blobs are color regions, not guaranteed individual game pieces.
"""
import math
import cv2
import numpy as np

KINDS = ("POLLEN", "RED_NECTAR", "BLUE_NECTAR")


def validate_config(c):
    if c['version'] != 1:
        raise ValueError('Unsupported configuration version')
    for key in ('width', 'height', 'max_blobs_per_color', 'max_age_ms'):
        if not isinstance(c[key], int) or c[key] <= 0:
            raise ValueError(key + ' must be a positive integer')
    k = c['morph_kernel']
    if not isinstance(k, int) or k < 1 or k > 15 or k % 2 == 0:
        raise ValueError('morph_kernel must be odd, between 1 and 15')
    for key in ('cluster_radius_inches', 'max_range_inches', 'max_aspect'):
        if not math.isfinite(c[key]) or c[key] <= 0:
            raise ValueError(key + ' must be finite and positive')
    for key in ('pollen_weight', 'nectar_weight', 'opponent_penalty',
                'distance_penalty_per_inch', 'settle_ms'):
        if not math.isfinite(c[key]) or c[key] < 0:
            raise ValueError(key + ' must be finite and nonnegative')
    if not 0 < c['min_area_fraction'] <= c['max_area_fraction'] <= 1:
        raise ValueError('Invalid area fractions')
    if not 0 <= c['min_fill'] <= 1:
        raise ValueError('Invalid min_fill')
    x0, y0, x1, y1 = c['roi']
    if not 0 <= x0 < x1 <= 1 or not 0 <= y0 < y1 <= 1:
        raise ValueError('roi must be [left, top, right, bottom] in normalized coordinates')
    for kind in KINDS:
        if not c['hsv'][kind]:
            raise ValueError('Missing HSV ranges for ' + kind)
        for r in c['hsv'][kind]:
            if len(r) != 6 or any(not isinstance(v, int) for v in r):
                raise ValueError('HSV ranges require six integers')
            if any(not 0 <= r[i] <= r[i + 3] <= (179 if i == 0 else 255) for i in range(3)):
                raise ValueError('Invalid HSV range')
    cal = c['calibration']
    if cal is not None:
        h = np.asarray(cal['homography'], dtype=float)
        poly = np.asarray(cal['polygon'], dtype=np.float32)
        if h.shape != (3, 3) or not np.isfinite(h).all() or abs(np.linalg.det(h)) < 1e-9:
            raise ValueError('Invalid homography')
        if poly.shape != (4, 2) or not np.isfinite(poly).all() or (poly < 0).any() or (poly > 1).any():
            raise ValueError('Calibration polygon requires four normalized corners')
        if not cv2.isContourConvex(poly) or abs(cv2.contourArea(poly)) < 0.001:
            raise ValueError('Calibration polygon must be convex, ordered, and nondegenerate')
        if not math.isfinite(cal['aspect']) or cal['aspect'] <= 0:
            raise ValueError('Invalid calibration aspect')
    return c


def project(u, v, c, aspect):
    cal = c['calibration']
    if cal is None or abs(aspect / cal['aspect'] - 1) > 0.01:
        return None
    if cv2.pointPolygonTest(np.asarray(cal['polygon'], np.float32), (u, v), False) < 0:
        return None
    q = np.asarray(cal['homography']) @ np.array([u, v, 1.0])
    if not np.isfinite(q).all() or abs(q[2]) < 1e-9:
        return None
    f, l = float(q[0] / q[2]), float(q[1] / q[2])
    if f < 0 or math.hypot(f, l) > c['max_range_inches']:
        return None
    return [f, l]


def field_position(position, pose):
    """pose = field x inches, y inches, CCW heading radians, +x at heading zero."""
    f, l = position
    x, y, h = pose
    return [x + f * math.cos(h) - l * math.sin(h),
            y + f * math.sin(h) + l * math.cos(h)]


def rank_targets(blobs, c, alliance='RED', mode='MIXED'):
    if alliance not in ('RED', 'BLUE') or mode not in ('POLLEN_ONLY', 'MIXED'):
        raise ValueError('Invalid alliance or mode')
    own = alliance + '_NECTAR'
    other = ('BLUE' if alliance == 'RED' else 'RED') + '_NECTAR'
    weights = {'POLLEN': c['pollen_weight'], own: c['nectar_weight'] if mode == 'MIXED' else 0,
               other: -c['opponent_penalty']}
    located = [b for b in blobs if b['position'] is not None]
    targets = []
    for index, b in enumerate(blobs):
        if b['position'] is None or weights[b['kind']] <= 0:
            continue
        members = [n for n in located if math.hypot(b['position'][0] - n['position'][0],
                                                   b['position'][1] - n['position'][1]) <= c['cluster_radius_inches']]
        counts = {k: sum(n['kind'] == k for n in members) for k in KINDS}
        score = sum(weights[k] * counts[k] for k in KINDS)
        score -= c['distance_penalty_per_inch'] * math.hypot(*b['position'])
        targets.append({'blob_index': index, 'position': b['position'], 'score': score, 'counts': counts})
    return sorted(targets, key=lambda t: (-t['score'], math.hypot(*t['position']), t['blob_index']))


def detect(image, c, intake_ready=False, alliance='RED', mode='MIXED'):
    """Input is BGR. Metric results require an explicit settled intake pose."""
    height, width = image.shape[:2]
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    x0, y0, x1, y1 = c['roi']
    left, top = int(x0 * width), int(y0 * height)
    right, bottom = min(width, int(x1 * width)), min(height, int(y1 * height))
    kernel = np.ones((c['morph_kernel'], c['morph_kernel']), np.uint8)
    blobs, masks = [], {}
    truncated = False
    for kind in KINDS:
        mask = np.zeros((height, width), np.uint8)
        for r in c['hsv'][kind]:
            mask |= cv2.inRange(hsv, np.array(r[:3]), np.array(r[3:]))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask[:top] = 0
        mask[bottom:] = 0
        mask[:, :left] = 0
        mask[:, right:] = 0
        masks[kind] = mask
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        accepted = []
        for contour in contours:
            area = cv2.contourArea(contour)
            x, y, w, h = cv2.boundingRect(contour)
            if not c['min_area_fraction'] <= area / (width * height) <= c['max_area_fraction']:
                continue
            if area / (w * h) < c['min_fill'] or max(w / h, h / w) > c['max_aspect']:
                continue
            # Midpoint of the lowest contour row approximates the floor contact point.
            pts = contour.reshape(-1, 2)
            lowest = pts[pts[:, 1] == pts[:, 1].max()]
            u, v = float(lowest[:, 0].mean()) / width, float(lowest[0, 1]) / height
            clipped = x <= left or y <= top or x + w >= right or y + h >= bottom
            position = project(u, v, c, width / height) if intake_ready and not clipped else None
            accepted.append({'kind': kind, 'box': [x, y, w, h], 'area': area,
                             'anchor': [u, v], 'position': position, 'clipped': clipped})
        accepted.sort(key=lambda b: (-b['area'], b['box'][0], b['box'][1]))
        truncated |= len(accepted) > c['max_blobs_per_color']
        blobs.extend(accepted[:c['max_blobs_per_color']])
    targets = rank_targets(blobs, c, alliance, mode) if intake_ready else []
    return {'blobs': blobs, 'targets': targets, 'best': targets[0] if targets else None,
            'counts': {k: sum(b['kind'] == k for b in blobs) for k in KINDS},
            'intake_ready': bool(intake_ready), 'truncated': truncated}, masks
