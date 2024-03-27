#!/usr/bin/env python3

import cv2
import os
from pathlib import Path


image_dir = Path(__file__).resolve().parents[1] / 'images'

if not image_dir.exists():
    image_dir.mkdir()

video_objects = [v[5:] for v in os.listdir(os.path.abspath("/dev")) if "video" in v]  # v[5:] removes 'video' from string
video_objects = [int(v) for v in video_objects if (int(v) % 2 == 0)]

for dev_id in video_objects:
    cap = cv2.VideoCapture(dev_id)
    ret, frame = cap.read()

    if ret:
        cv2.imwrite(f'{str(image_dir)}/Camera_{dev_id}.png', frame)
    
    cap.release()