#!/usr/bin/env python3
"""Create a synthetic replay and calibration for checking the entire tester without a camera."""
import json
from pathlib import Path
import cv2
import numpy as np
from run import DEFAULT_CONFIG


def main():
    out = Path(__file__).resolve().parent / 'output'
    out.mkdir(exist_ok=True)
    config = json.loads(DEFAULT_CONFIG.read_text())
    config['calibration'] = {'homography': [[0, -60, 70], [-60, 0, 30], [0, 0, 1]],
                             'polygon': [[.05,.05],[.95,.05],[.95,.95],[.05,.95]], 'aspect': 4/3}
    (out / 'demo-config.json').write_text(json.dumps(config, indent=2) + '\n')
    video = cv2.VideoWriter(str(out / 'demo.avi'), cv2.VideoWriter_fourcc(*'MJPG'), 20, (640, 480))
    if not video.isOpened():
        raise RuntimeError('Cannot open MJPG video writer')
    try:
        for i in range(120):
            frame = np.full((480, 640, 3), 35, np.uint8)
            shift = int(15*np.sin(i/15))
            for x, y, color in [(140,310,(0,255,255)),(200,300,(0,255,255)),
                                (170,240,(0,0,255)),(480,160,(255,0,0)),(540,210,(255,0,0))]:
                cv2.circle(frame, (x+shift,y), 22, color, -1)
            video.write(frame)
            if i == 0: cv2.imwrite(str(out / 'demo.png'), frame)
    finally:
        video.release()
    print('Created', out / 'demo.avi', 'and demo-config.json (synthetic calibration only).')


if __name__ == '__main__': main()
