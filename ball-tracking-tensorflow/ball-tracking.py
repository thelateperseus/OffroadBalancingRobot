# Copyright 2021 The TensorFlow Authors. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""Main script to run the object detection routine."""
import argparse
import sys
import serial
import signal
import time
from collections import deque

import cv2
from object_detector import Detection
from object_detector import ObjectDetector
from object_detector import ObjectDetectorOptions
from imutils.video import VideoStream
from pan_tilt_hat import PanTiltHat
from simple_pid import PID
import util

arduino = serial.Serial(port='/dev/ttyACM0', baudrate=115200, timeout=0.001)

def weighted_score(detection: Detection) -> float:
    box = detection.bounding_box
    width = box.right - box.left
    height = box.bottom - box.top
    area = width * height
    squareness = min(width, height) / max(width, height)
    return area * squareness * detection.categories[0].score # Only one detection category

def run(model: str, camera_id: int, width: int, height: int, num_threads: int, output_file: str) -> None:
  """Continuously run inference on images acquired from the camera.
  Args:
    model: Name of the TFLite object detection model.
    camera_id: The camera id to be passed to OpenCV.
    width: The width of the frame captured from the camera.
    height: The height of the frame captured from the camera.
    num_threads: The number of CPU threads to run the model.
    output: File name of the recording output.
  """

  pth = PanTiltHat()
  pth.pan(0)
  pth.tilt(20)

  pidPan = PID(0.1, 0.5, 0.0, setpoint=160, sample_time=None, output_limits=(-75, 75), auto_mode=False)
  pidTilt = PID(0.075, 0.375, 0.0, setpoint=120, sample_time=None, output_limits=(-75, 30), auto_mode=False)
  pidSteering = PID(1.5, 0.25, 0.0, setpoint=0, sample_time=None, output_limits=(-60, 60), auto_mode=False)
  pidSpeed = PID(0.5, 0, 0.0, setpoint=40, sample_time=None, output_limits=(-5, 5), auto_mode=False)

  # Variables to calculate FPS
  start_time = time.time()
  fps_avg_frame_count = 10
  frame_times = deque([], fps_avg_frame_count)

  # Start capturing video input from the camera
  cap = VideoStream(src=0, usePiCamera=True, resolution=(width,height), framerate=20).start()
  codec = cv2.VideoWriter_fourcc(*'MJPG')
  writer = cv2.VideoWriter(output_file, codec, 10, (width,height))
  print(output_file)

  # Visualization parameters
  row_size = 20  # pixels
  left_margin = 24  # pixels
  text_color = (0, 0, 255)  # red
  font_size = 1
  font_thickness = 1

  # Initialize the object detection model
  options = ObjectDetectorOptions(
      num_threads=num_threads,
      score_threshold=0.3,
      max_results=3)
  detector = ObjectDetector(model_path=model, options=options)

  def signal_handler(sig, frame):
    cap.stop()
    writer.release()
    # cv2.destroyAllWindows()
    pth.shutdown()
    sys.exit(0)

  signal.signal(signal.SIGINT, signal_handler)

  # Continuously capture images from the camera and run inference
  while True:
    image = cap.read()
    if image is None:
      # allow the camera or video file to warm up
      time.sleep(0.1)
      continue

    # Run object detection estimation using the model.
    detections = detector.detect(image)

    # Draw keypoints and edges on input image
    image = util.visualize(image, detections)

    # Calculate the FPS
    end_time = time.time()
    frame_times.append(end_time - start_time)
    avg_frame_time = sum(frame_times) / len(frame_times)
    fps = 1.0 / avg_frame_time
    start_time = time.time()

    # Show the FPS
    fps_text = '{:2.1f} FPS'.format(fps)
    text_location = (left_margin, row_size)
    cv2.putText(image, fps_text, text_location, cv2.FONT_HERSHEY_PLAIN,
                font_size, text_color, font_thickness)

    # Stop the program if the ESC key is pressed.
    # if cv2.waitKey(1) == 27:
    #   break
    # cv2.imshow('object_detector', image)
    writer.write(image)
    detection = max(detections, key=weighted_score) if detections else None
    width = detection.bounding_box.right - detection.bounding_box.left if detections else None
    height = detection.bounding_box.bottom - detection.bounding_box.top if detections else None
    x = detection.bounding_box.left + width / 2 if detections else None
    y = detection.bounding_box.top + height / 2 if detections else None
    print(f'{fps_text}   x: {x}, y: {y}, w: {width}')
    if len(detections) > 1:
      print(f'{detections} weighted_scores: {list(map(lambda x: round(weighted_score(x), 1), detections))}')

    if x is None:
      # Reset PID controllers if ball not found
      pidPan.set_auto_mode(False)
      pidTilt.set_auto_mode(False)
      pidSteering.set_auto_mode(False)
      pidSpeed.set_auto_mode(False)

    else:
      pidPan.set_auto_mode(True, last_output=pidPan._last_output)
      pidTilt.set_auto_mode(True, last_output=pidTilt._last_output)
      pidSteering.set_auto_mode(True, last_output=0)
      pidSpeed.set_auto_mode(True, last_output=0)
      panAngle = pidPan(x)
      tiltAngle = pidTilt(240 - y)
      steeringValue = -pidSteering(panAngle)
      speedValue = pidSpeed(width)
      print(f'panAngle: {panAngle:.1f}, tiltAngle: {tiltAngle:.1f}, speed: {speedValue:.1f}, steering: {steeringValue:.1f}')
      pth.pan(panAngle)
      pth.tilt(tiltAngle)
      serialCommand = f'{speedValue:.1f} {steeringValue:.1f}\r\n'
      arduino.write(bytes(serialCommand, 'utf-8'))

  signal_handler()


def main():
  parser = argparse.ArgumentParser(
      formatter_class=argparse.ArgumentDefaultsHelpFormatter)
  parser.add_argument(
      '--model',
      help='Path of the object detection model.',
      required=False,
      default='efficientdet_lite0_ball.tflite')
  parser.add_argument(
      '--cameraId', help='Id of camera.', required=False, type=int, default=0)
  parser.add_argument(
      '--frameWidth',
      help='Width of frame to capture from camera.',
      required=False,
      type=int,
      default=320)
  parser.add_argument(
      '--frameHeight',
      help='Height of frame to capture from camera.',
      required=False,
      type=int,
      default=240)
  parser.add_argument(
      '--numThreads',
      help='Number of CPU threads to run the model.',
      required=False,
      type=int,
      default=4)
  parser.add_argument(
      '--output',
      help='File name of the recording output.',
      required=False,
      default='output.avi')
  args = parser.parse_args()

  run(args.model, int(args.cameraId), args.frameWidth, args.frameHeight,
      int(args.numThreads), args.output)


if __name__ == '__main__':
  main()
