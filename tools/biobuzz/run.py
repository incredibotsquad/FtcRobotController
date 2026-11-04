#!/usr/bin/env python3
"""USB camera viewer, HSV tuner, calibration, image/video replay and JSON output."""
import argparse
import json
import time
from pathlib import Path
import cv2
import numpy as np
from core import KINDS, detect, validate_config

DEFAULT_CONFIG = Path(__file__).resolve().parents[2] / 'TeamCode/src/main/assets/biobuzz.json'
COLORS = {'POLLEN': (0, 255, 255), 'RED_NECTAR': (0, 0, 255), 'BLUE_NECTAR': (255, 100, 0)}


def calibrate(frame, config, output):
    print('Enter four measured floor positions in perimeter order (not a crossing order).')
    print('Each position: forward,left in INCHES from robot origin. Left is positive.')
    print('Example order: near-left, far-left, far-right, near-right. Click in the SAME order.')
    world = []
    for i in range(4):
        pair = [float(v) for v in input('Floor point %d forward,left: ' % (i + 1)).split(',')]
        if len(pair) != 2 or not np.isfinite(pair).all():
            raise ValueError('Expected two finite numbers')
        world.append(pair)
    world = np.asarray(world, np.float32)
    if not cv2.isContourConvex(world) or abs(cv2.contourArea(world)) < 1:
        raise ValueError('Floor points must form a convex quadrilateral in perimeter order')
    points = []
    def click(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN and len(points) < 4:
            points.append([x / frame.shape[1], y / frame.shape[0]])
    cv2.namedWindow('Calibration')
    cv2.setMouseCallback('Calibration', click)
    while True:
        preview = frame.copy()
        for i, (u, v) in enumerate(points):
            p = (int(u * frame.shape[1]), int(v * frame.shape[0]))
            cv2.circle(preview, p, 5, (0, 255, 0), -1)
            cv2.putText(preview, str(i + 1), p, 0, 0.7, (255, 255, 255), 2)
        cv2.putText(preview, 'Click 4 floor marks; ENTER save; R reset; ESC cancel', (10, 25), 0, .5, (255, 255, 255), 1)
        cv2.imshow('Calibration', preview)
        key = cv2.waitKey(20) & 255
        if key == 27:
            break
        if key == ord('r'):
            points.clear()
        if key in (10, 13) and len(points) == 4:
            h = cv2.getPerspectiveTransform(np.asarray(points, np.float32), world)
            candidate = dict(config)
            candidate['calibration'] = {'homography': h.tolist(), 'polygon': points,
                                        'aspect': frame.shape[1] / frame.shape[0]}
            validate_config(candidate)
            output.parent.mkdir(parents=True, exist_ok=True)
            output.write_text(json.dumps(candidate, indent=2) + '\n')
            config.update(candidate)
            print('Saved calibration to', output)
            break
    cv2.destroyWindow('Calibration')


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config', type=Path, default=DEFAULT_CONFIG)
    p.add_argument('--camera', type=int, default=0)
    p.add_argument('--video', type=str)
    p.add_argument('--image', type=str)
    p.add_argument('--alliance', choices=['RED', 'BLUE'], default='RED')
    p.add_argument('--mode', choices=['POLLEN_ONLY', 'MIXED'], default='MIXED')
    p.add_argument('--intake', action='store_true', help='Confirm camera is stationary at calibrated intake pose')
    p.add_argument('--headless', action='store_true')
    p.add_argument('--frames', type=int, default=0, help='0 runs until quit/end of video')
    p.add_argument('--jsonl', type=Path, help='Write one JSON result per frame')
    p.add_argument('--output', type=Path, default=Path('tools/biobuzz/output'))
    args = p.parse_args()
    if args.image and args.video:
        p.error('Choose image or video')
    c = validate_config(json.loads(args.config.read_text()))
    cap = None
    still = cv2.imread(args.image) if args.image else None
    if args.image and still is None:
        p.error('Cannot read image: ' + args.image)
    if still is None:
        cap = cv2.VideoCapture(args.video if args.video else args.camera)
        if not cap.isOpened():
            p.error('Cannot open camera/video. Check camera index and macOS Camera permission for Terminal/Codex.')
        if not args.video:
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, c['width'])
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, c['height'])
    args.output.mkdir(parents=True, exist_ok=True)
    log = None
    if args.jsonl:
        args.jsonl.parent.mkdir(parents=True, exist_ok=True)
        log = args.jsonl.open('w')
    tuning, selected = False, 0
    intake = args.intake
    # A saved frame/video can represent an already-settled pose. Do not consume
    # an entire fast headless replay while waiting on a physical servo timer.
    ready_at = 0 if args.image or args.video else time.monotonic() + c['settle_ms'] / 1000
    frame_count = 0
    last_frame = None
    def sample(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN and last_frame is not None:
            value = cv2.cvtColor(last_frame, cv2.COLOR_BGR2HSV)[y, x].tolist()
            print('Clicked HSV:', value, '| edit hsv ranges in config or use T tuner')
    if not args.headless:
        cv2.namedWindow('BIOBuzz')
        cv2.setMouseCallback('BIOBuzz', sample)
    print('Q quit | I intake/hive | A alliance | M pickup mode | C calibrate | T HSV tuner | 1/2/3 color | S save')
    try:
        while True:
            ok, frame = (True, still.copy()) if still is not None else cap.read()
            if not ok:
                if not args.video:
                    raise RuntimeError('Camera stopped returning frames; no target is available')
                break
            last_frame = frame.copy()
            if tuning:
                r = c['hsv'][KINDS[selected]][0]
                values = [cv2.getTrackbarPos(n, 'HSV tuner') for n in ('H low', 'S low', 'V low', 'H high', 'S high', 'V high')]
                if all(values[i] <= values[i + 3] for i in range(3)):
                    r[:] = values
            start = time.monotonic()
            result, masks = detect(frame, c, intake and start >= ready_at, args.alliance, args.mode)
            result['processing_ms'] = (time.monotonic() - start) * 1000
            result['frame'] = frame_count
            if log:
                log.write(json.dumps(result, allow_nan=False) + '\n')
                log.flush()
            label_boxes = []
            for b in result['blobs']:
                x, y, w, h = b['box']
                cv2.rectangle(frame, (x, y), (x+w, y+h), COLORS[b['kind']], 2)
                label = b['kind']
                if b['position']:
                    label += ' %.1ff %.1fl in' % tuple(b['position'])
                (tw, th), baseline = cv2.getTextSize(label, 0, .4, 1)
                tx = max(0, min(x, frame.shape[1] - tw - 2))
                ty = max(60, y - 5)
                while ty > 65 and any(tx < bx + bw and tx + tw > bx and
                                     ty - th < by + bh and ty + baseline > by
                                     for bx, by, bw, bh in label_boxes):
                    ty -= th + baseline + 3
                label_boxes.append((tx, ty-th, tw, th+baseline))
                cv2.putText(frame, label, (tx, ty), 0, .4, COLORS[b['kind']], 1)
            if result['best']:
                b = result['blobs'][result['best']['blob_index']]
                u, v = b['anchor']
                cv2.circle(frame, (int(u*frame.shape[1]), int(v*frame.shape[0])), 12, (0, 255, 0), 2)
            status = 'INTAKE' if intake else 'HIVE / MOVING: no positions'
            if c['calibration'] is None:
                status += ' | UNCALIBRATED: press C'
            elif abs(frame.shape[1] / frame.shape[0] / c['calibration']['aspect'] - 1) > .01:
                status += ' | ASPECT MISMATCH: recalibrate'
            cv2.putText(frame, status, (10, 20), 0, .48, (255, 255, 255), 1)
            cv2.putText(frame, '%s %s blobs P/R/B %d/%d/%d | %.1f ms' %
                        (args.alliance, args.mode, *(result['counts'][k] for k in KINDS), result['processing_ms']),
                        (10, 42), 0, .45, (255, 255, 255), 1)
            if args.headless:
                if frame_count == 0:
                    cv2.imwrite(str(args.output / 'preview.jpg'), frame)
                key = -1
            else:
                cv2.imshow('BIOBuzz', frame)
                if tuning:
                    cv2.imshow('Mask', masks[KINDS[selected]])
                key = cv2.waitKey(1 if still is None else 30) & 255
            frame_count += 1
            if key == ord('q') or (args.frames > 0 and frame_count >= args.frames) or (args.headless and still is not None):
                break
            if key == ord('i'):
                intake = not intake
                ready_at = time.monotonic() + c['settle_ms'] / 1000
            if key == ord('a'):
                args.alliance = 'BLUE' if args.alliance == 'RED' else 'RED'
            if key == ord('m'):
                args.mode = 'POLLEN_ONLY' if args.mode == 'MIXED' else 'MIXED'
            if key == ord('c'):
                calibrate(last_frame, c, args.config)
                intake = False  # Explicitly re-enable after confirming the mount has not moved.
            if key == ord('s'):
                args.config.write_text(json.dumps(c, indent=2) + '\n')
                cv2.imwrite(str(args.output / ('frame-%d.png' % frame_count)), last_frame)
                print('Saved config and raw frame')
            if key in (ord('t'), ord('1'), ord('2'), ord('3')):
                if key != ord('t'):
                    selected = key - ord('1')
                tuning = True
                cv2.namedWindow('HSV tuner')
                for i, n in enumerate(('H low', 'S low', 'V low', 'H high', 'S high', 'V high')):
                    cv2.createTrackbar(n, 'HSV tuner', c['hsv'][KINDS[selected]][0][i], 179 if i % 3 == 0 else 255, lambda _: None)
                    cv2.setTrackbarPos(n, 'HSV tuner', c['hsv'][KINDS[selected]][0][i])
                print('Tuning', KINDS[selected], 'first range; red wraparound second range is editable in JSON')
    finally:
        if cap:
            cap.release()
        if log:
            log.close()
        if not args.headless:
            cv2.destroyAllWindows()
    print('Processed', frame_count, 'frames. Output:', args.output)


if __name__ == '__main__':
    main()
