#!/bin/zsh
set -e
cd "$(dirname "$0")/../.."
if [[ ! -x tools/biobuzz/.venv/bin/python ]]; then
  python3 -m venv tools/biobuzz/.venv
fi
if ! tools/biobuzz/.venv/bin/python -c 'import cv2, numpy' 2>/dev/null; then
  tools/biobuzz/.venv/bin/python -m pip install -r tools/biobuzz/requirements.txt
fi
tools/biobuzz/.venv/bin/python tools/biobuzz/run.py "$@"
