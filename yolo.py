# Inferent best.pt --img 640 --conf 0.25 --source 0  # webcam

from ultralytics import YOLO
import cv2
import numpy as np
import time

model = YOLO('best.pt')
cap = cv2.VideoCapture(0)
while True:
    ret, frame = cap.read()
    if not ret:
        break

    results = model(frame)

    for result in results:
        annotated_frame = result.plot()
        cv2.imshow('YOLOv8 Detection', annotated_frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
cap.release()
cv2.destroyAllWindows()