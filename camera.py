# import the necessary packages
#from picamera.array import PiRGBArray
#from picamera import PiCamera
#import time
##import cv2
#
## initialize the camera and grab a reference to the raw camera capture
#camera = picamera.PiCamera()
#camera.resolution = (640, 480)
#camera.framerate = 32
##rawCapture = PiRGBArray(camera, size=(640, 480))
#
## allow the camera to warmup
#time.sleep(0.1)

# capture frames from the camera
#for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
#	# grab the raw NumPy array representing the image, then initialize the timestamp
#	# and occupied/unoccupied text
#	image = frame.array
#
#	# show the frame
#	cv2.imshow("Frame", image)
#	key = cv2.waitKey(1) & 0xFF
#
#	# clear the stream in preparation for the next frame
#	rawCapture.truncate(0)
#
#	# if the `q` key was pressed, break from the loop
#	if key == ord("q"):
#		break
import io
import picamera
import logging
import socketserver
from threading import Condition, Thread
from http import server

PAGE="""\
<html>
<head>
<title>Car Front Camera</title>
</head>
<body>
<h1>Car Front Camera</h1>
<img src="stream.mjpg" width="1080" height="720" />
</body>
</html>
"""

class StreamingOutput(object):
    def __init__(self):
        self.frame = None
        self.buffer = io.BytesIO()
        self.condition = Condition()

    def write(self, buf):
        if buf.startswith(b'\xff\xd8'):
            self.buffer.truncate()
            with self.condition:
                self.frame = self.buffer.getvalue()
                self.condition.notify_all()
            self.buffer.seek(0)
        return self.buffer.write(buf)

class StreamingHandler(server.BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == '/':
            self.send_response(301)
            self.send_header('Location', '/index.html')
            self.end_headers()
        elif self.path == '/index.html':
            content = PAGE.encode('utf-8')
            self.send_response(200)
            self.send_header('Content-Type', 'text/html')
            self.send_header('Content-Length', len(content))
            self.end_headers()
            self.wfile.write(content)
        elif self.path == '/stream.mjpg':
            self.send_response(200)
            self.send_header('Age', 0)
            self.send_header('Cache-Control', 'no-cache, private')
            self.send_header('Pragma', 'no-cache')
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=FRAME')
            self.end_headers()
            try:
                while True:
                    with output.condition:
                        output.condition.wait()
                        frame = output.frame
                    self.wfile.write(b'--FRAME\r\n')
                    self.send_header('Content-Type', 'image/jpeg')
                    self.send_header('Content-Length', len(frame))
                    self.end_headers()
                    self.wfile.write(frame)
                    self.wfile.write(b'\r\n')
            except Exception as e:
                logging.warning(
                    'Removed streaming client %s: %s',
                    self.client_address, str(e))
        else:
            self.send_error(404)
            self.end_headers()

class StreamingServer(socketserver.ThreadingMixIn, server.HTTPServer):
    allow_reuse_address = True
    daemon_threads = True


def webcam_recording(output):
    camera = picamera.PiCamera(resolution='1080x720', framerate=24)
    camera.start_recording(output, format='mjpeg')
    try:
        address = ('', 8000)
        server = StreamingServer(address, StreamingHandler)
        server.serve_forever()
    finally:
        camera.stop_recording()#

output = StreamingOutput()

if __name__ == '__main__':
    output = StreamingOutput()
    webcam_recording(output)
