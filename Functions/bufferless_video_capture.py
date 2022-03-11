"""
https://stackoverflow.com/questions/43665208/how-to-get-the-latest-frame-from-capture-device-camera-in-opencv

Credit to Ulrich Stern for original class. Modifications made as necessary for this project.
"""

import cv2, queue, threading, logging, atexit

class BufferlessVideoCapture:
  def __init__(self, name):
    self.cap = cv2.VideoCapture(name)
    if not self.cap.isOpened():
      logging.error("Failed to open capture device.")
    self.q = queue.Queue()
    t = threading.Thread(target=self._reader)
    t.daemon = True
    t.start()
    atexit.register(self.cleanup)

  # read frames as soon as they are available, keeping only most recent one
  def _reader(self):
    while True:
      ret, frame = self.cap.read()
      if not ret:
        break
      if not self.q.empty():
        try:
          self.q.get_nowait()   # discard previous (unprocessed) frame
        except queue.Empty:
          pass
      self.q.put(frame)

  def read(self):
    return self.q.get()

  def cleanup(self):
    self.cap.release()

"""Test the class if this script is run directly."""
if __name__ == "__main__":
  import time
  cap = BufferlessVideoCapture(2)
  while True:
    time.sleep(.5)   # simulate time between events
    frame = cap.read()
    cv2.imshow("frame", frame)
    if chr(cv2.waitKey(1)&255) == 'q':
      break
