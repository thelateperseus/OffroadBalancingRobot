# import the necessary packages
from imutils.video import VideoStream
from pantilthat import PanTiltHat
import numpy as np
import cv2
import signal
import sys
import time

pth = PanTiltHat()
pth.pan(0)
pth.tilt(20)

vs = VideoStream(src=0, usePiCamera=True, resolution=(320,240), framerate=25).start()

# allow the camera to warm up
time.sleep(2.0)

#codec = cv2.VideoWriter_fourcc(*'mp4v')
#out = cv2.VideoWriter("output.mp4", codec, 25, (320,240))
codec = cv2.VideoWriter_fourcc(*'MJPG')
out = cv2.VideoWriter("output.avi", codec, 25, (320,240))

def signal_handler(sig, frame):
    vs.stop()
    out.release()
    cv2.destroyAllWindows()
    pth.shutdown()
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

# keep looping
while True:
    startTime = time.time()

    # grab the current frame
    frame = vs.read()

    out.write(frame)

    # show the frame to our screen
    #cv2.imshow("Frame", frame)
    #key = cv2.waitKey(1) & 0xFF

    # if the 'q' key is pressed, stop the loop
    #if key == ord("q"):
    #    break
