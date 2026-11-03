#!/usr/bin/env python3
"""Bundle the tested Python implementation and configuration into one SnapScript."""
import argparse
import json
from pathlib import Path
from core import validate_config
from run import DEFAULT_CONFIG

TAIL = '''
_sequence = 0

def runPipeline(image, llrobot):
    global _sequence
    _sequence = (_sequence + 1) % 1000000
    # Inputs: request ID, intake ready, blue alliance, pollen-only.
    request = int(llrobot[0]) if len(llrobot) >= 4 else 0
    ready = len(llrobot) >= 4 and llrobot[1] == 1 and request > 0
    alliance = 'BLUE' if len(llrobot) >= 4 and llrobot[2] == 1 else 'RED'
    mode = 'POLLEN_ONLY' if len(llrobot) >= 4 and llrobot[3] == 1 else 'MIXED'
    result, _ = detect(image, CONFIG, ready, alliance, mode)
    best = result['best']
    counts = result['counts']
    # Positive echoed request means a usable best target; negative means no target.
    output = [request if best else -request, 0, 0, 0,
              counts['POLLEN'], counts['RED_NECTAR'], counts['BLUE_NECTAR'], _sequence]
    contour = np.empty((0, 1, 2), dtype=np.int32)
    if best:
        output[1:4] = [best['position'][0], best['position'][1], best['score']]
        b = result['blobs'][best['blob_index']]
        x, y, w, h = b['box']
        contour = np.array([[[x,y]], [[x+w-1,y]], [[x+w-1,y+h-1]], [[x,y+h-1]]], dtype=np.int32)
        cv2.drawContours(image, [contour], -1, (0,255,0), 2)
    return contour, image, output
'''


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config', type=Path, default=DEFAULT_CONFIG)
    p.add_argument('--output', type=Path, default=Path('tools/biobuzz/output/limelight_pipeline.py'))
    args = p.parse_args()
    config = validate_config(json.loads(args.config.read_text()))
    source = Path(__file__).with_name('core.py').read_text()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(source + '\nCONFIG = ' + repr(config) + '\n' + TAIL)
    print('Upload to a dedicated Python SnapScript pipeline:', args.output)
    if config['calibration'] is None:
        print('Uncalibrated: blob counts work, metric targets remain unavailable.')


if __name__ == '__main__':
    main()
