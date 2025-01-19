import asyncio
from mavsdk import System
import pygame
import cv2
import cv2.aruco as aruco
import gi
import numpy as np
from datetime import datetime

gi.require_version('Gst', '1.0')
from gi.repository import Gst

# ARUCO TAG DEFINITION
id_to_find = 72
marker_size = 10  # [cm]

class Video():
    def __init__(self, port=5600):
        Gst.init(None)
        self.port = port
        self._frame = None
        self.video_source = f'udpsrc port={self.port} ! queue max-size-buffers=3'
        self.video_codec = '! application/x-rtp, payload=96 ! rtph264depay ! h264parse ! avdec_h264'
        self.video_decode = '! queue ! decodebin ! videoconvert ! video/x-raw,format=(string)BGR ! videoconvert'
        self.video_sink_conf = '! appsink emit-signals=true sync=false max-buffers=1 drop=true'
        self.video_pipe = None
        self.video_sink = None
        self.run()

    def start_gst(self, config=None):
        if not config:
            return
        command = ' '.join(config)
        self.video_pipe = Gst.parse_launch(command)
        self.video_pipe.set_state(Gst.State.PLAYING)
        self.video_sink = self.video_pipe.get_by_name('appsink0')

    @staticmethod
    def gst_to_opencv(sample):
        buf = sample.get_buffer()
        caps = sample.get_caps()
        array = np.ndarray(
            (
                caps.get_structure(0).get_value('height'),
                caps.get_structure(0).get_value('width'),
                3
            ),
            buffer=buf.extract_dup(0, buf.get_size()), dtype=np.uint8)
        return array

    def frame(self):
        return self._frame

    def frame_available(self):
        return self._frame is not None

    def run(self):
        self.start_gst([
            self.video_source,
            self.video_codec,
            self.video_decode,
            self.video_sink_conf
        ])
        self.video_sink.connect('new-sample', self.callback)

    def callback(self, sink):
        sample = sink.emit('pull-sample')
        new_frame = self.gst_to_opencv(sample)
        self._frame = new_frame
        return Gst.FlowReturn.OK

class ArucoDetector:
    def __init__(self):
        # Initialize ArUco detector
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
        self.parameters = aruco.DetectorParameters()
        self.detector = aruco.ArucoDetector(self.aruco_dict, self.parameters)

    def detect_marker(self, frame):
        # Make copy of frame
        frame_copy = frame.copy()
        gray = cv2.cvtColor(frame_copy, cv2.COLOR_BGR2GRAY)
        
        # Detect markers
        corners, ids, rejected = self.detector.detectMarkers(gray)
        
        # Draw ALL detected markers and print their IDs
        if ids is not None:
            print(f"\rDetected ArUco markers with IDs: {ids.flatten()}", end='')
            aruco.drawDetectedMarkers(frame_copy, corners, ids)
            
            # Add text showing detected IDs
            id_text = f"Detected IDs: {ids.flatten()}"
            cv2.putText(frame_copy, id_text, 
                       (10, 30), cv2.FONT_HERSHEY_PLAIN, 1, (0, 255, 0), 2)
        else:
            cv2.putText(frame_copy, "No markers detected", 
                       (10, 30), cv2.FONT_HERSHEY_PLAIN, 1, (0, 255, 0), 2)
        
        return frame_copy

async def main():
    # Initialize video and detector
    video = Video()
    detector = ArucoDetector()
    
    # Initialize drone connection
    drone = System()
    await drone.connect(system_address="udp://:14540")
    
    print("Waiting for drone connection...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Drone connected!")
            break
    
    print("-- Arming")
    await drone.action.arm()
    print("-- Armed and ready for detection test")
    
    # Main detection loop
    while True:
        if video.frame_available():
            frame = video.frame()
            
            # Process frame and detect markers
            processed_frame = detector.detect_marker(frame)
            
            # Display frame
            cv2.imshow("Drone Camera Feed", processed_frame)
        
        # Check for exit (press 'q' in video window)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            print("\n-- Disarming")
            await drone.action.disarm()
            break
        
        await asyncio.sleep(0.1)
    
    cv2.destroyAllWindows()

def init_pygame():
    pygame.init()
    pygame.display.set_mode((400, 400))

if __name__ == "__main__":
    init_pygame()
    loop = asyncio.get_event_loop()
    loop.run_until_complete(main())