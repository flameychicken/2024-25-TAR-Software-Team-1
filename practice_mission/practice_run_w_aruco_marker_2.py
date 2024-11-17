import asyncio
from mavsdk import System
from mavsdk.offboard import VelocityBodyYawspeed
import pygame
import cv2
import cv2.aruco as aruco
import gi
import numpy as np
import sys, time, math
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
        self.frame_count = 0
        self.last_frame_time = time.time()
        self.fps = 0

        # Modified pipeline with queue elements for better buffering
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
        
        # Calculate FPS
        current_time = time.time()
        self.frame_count += 1
        if current_time - self.last_frame_time >= 1.0:
            self.fps = self.frame_count
            self.frame_count = 0
            self.last_frame_time = current_time
            
        self._frame = new_frame
        return Gst.FlowReturn.OK

class ArucoDetector:
    def __init__(self):
        # Load camera calibration
        calib_path = "./opencv/camera_calibration/"
        self.camera_matrix = np.loadtxt(calib_path + 'cameraMatrix_webcam.txt', delimiter=',')
        self.camera_distortion = np.loadtxt(calib_path + 'cameraDistortion_webcam.txt', delimiter=',')
        
        # Initialize ArUco detector
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
        self.parameters = aruco.DetectorParameters()
        self.detector = aruco.ArucoDetector(self.aruco_dict, self.parameters)
        
        # Initialize debug information
        self.last_detection_time = None
        self.detection_count = 0
        self.frames_processed = 0

    def detect_marker(self, frame):
        self.frames_processed += 1
        
        # Make copy of frame
        frame_copy = frame.copy()
        gray = cv2.cvtColor(frame_copy, cv2.COLOR_BGR2GRAY)
        
        # Detect markers
        corners, ids, rejected = self.detector.detectMarkers(gray)
        
        marker_info = {
            'detected': False,
            'position': None,
            'attitude': None,
            'frame': frame_copy
        }
        
        if ids is not None and id_to_find in ids:
            self.detection_count += 1
            self.last_detection_time = datetime.now()
            
            # Get the index of our target marker
            marker_idx = np.where(ids == id_to_find)[0][0]
            
            # Estimate pose
            ret = aruco.estimatePoseSingleMarkers(
                [corners[marker_idx]], marker_size, 
                self.camera_matrix, self.camera_distortion
            )
            rvec, tvec = ret[0][0, 0, :], ret[1][0, 0, :]
            
            # Draw the marker
            aruco.drawDetectedMarkers(frame_copy, corners)
            cv2.drawFrameAxes(frame_copy, self.camera_matrix, 
                            self.camera_distortion, rvec, tvec, 10)
            
            # Calculate position and attitude
            marker_info['detected'] = True
            marker_info['position'] = tvec
            marker_info['attitude'] = self.get_euler_angles(rvec)
            
            # Draw debug information
            self.draw_debug_info(frame_copy, tvec, marker_info['attitude'])
            
        marker_info['frame'] = frame_copy
        return marker_info

    def get_euler_angles(self, rvec):
        R_ct = np.matrix(cv2.Rodrigues(rvec)[0])
        R_tc = R_ct.T
        
        # Get euler angles
        roll, pitch, yaw = self.rotation_matrix_to_euler_angles(R_tc)
        return np.array([roll, pitch, yaw])

    @staticmethod
    def rotation_matrix_to_euler_angles(R):
        sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])
        singular = sy < 1e-6

        if not singular:
            x = math.atan2(R[2, 1], R[2, 2])
            y = math.atan2(-R[2, 0], sy)
            z = math.atan2(R[1, 0], R[0, 0])
        else:
            x = math.atan2(-R[1, 2], R[1, 1])
            y = math.atan2(-R[2, 0], sy)
            z = 0

        return np.array([x, y, z])

    def draw_debug_info(self, frame, position, attitude):
        font = cv2.FONT_HERSHEY_PLAIN
        
        # Draw position
        pos_text = f"Marker Position: x={position[0]:.1f} y={position[1]:.1f} z={position[2]:.1f}"
        cv2.putText(frame, pos_text, (10, 30), font, 1, (0, 255, 0), 2)
        
        # Draw attitude
        att_text = f"Marker Attitude: r={math.degrees(attitude[0]):.1f} p={math.degrees(attitude[1]):.1f} y={math.degrees(attitude[2]):.1f}"
        cv2.putText(frame, att_text, (10, 60), font, 1, (0, 255, 0), 2)
        
        # Draw detection stats
        detection_rate = (self.detection_count / self.frames_processed * 100) if self.frames_processed > 0 else 0
        stats_text = f"Detection rate: {detection_rate:.1f}% ({self.detection_count}/{self.frames_processed})"
        cv2.putText(frame, stats_text, (10, 90), font, 1, (0, 255, 0), 2)

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
    
    # Main loop
    while True:
        if video.frame_available():
            frame = video.frame()
            
            # Process frame and detect marker
            marker_info = detector.detect_marker(frame)
            
            # Display frame with debug info
            cv2.putText(marker_info['frame'], f"FPS: {video.fps}", 
                       (10, 120), cv2.FONT_HERSHEY_PLAIN, 1, (0, 255, 0), 2)
            cv2.imshow("Drone Camera Feed", marker_info['frame'])
            
            # Print detection status
            if marker_info['detected']:
                print(f"\rMarker detected! Position: {marker_info['position']}", end='')
            else:
                print("\rSearching for marker...", end='')
            
        # Check for exit
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        
        await asyncio.sleep(0.01)  # Small sleep to prevent CPU overload
    
    cv2.destroyAllWindows()

if __name__ == "__main__":
    loop = asyncio.get_event_loop()
    loop.run_until_complete(main())