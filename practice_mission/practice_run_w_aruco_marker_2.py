import asyncio
from mavsdk import System
<<<<<<< HEAD
from mavsdk.offboard import (
    VelocityBodyYawspeed,
    PositionNedYaw,
    OffboardError
)
=======
from mavsdk.offboard import VelocityBodyYawspeed, OffboardError, PositionNedYaw
>>>>>>> fe249176552b0ab121813534f0273a75ff685664
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
        
<<<<<<< HEAD
        # Calculate FPS
=======
>>>>>>> fe249176552b0ab121813534f0273a75ff685664
        current_time = time.time()
        self.frame_count += 1
        if current_time - self.last_frame_time >= 1.0:
            self.fps = self.frame_count
            self.frame_count = 0
            self.last_frame_time = current_time
            
        self._frame = new_frame
        return Gst.FlowReturn.OK

class ArucoDetector:
<<<<<<< HEAD
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

async def takeoff_and_hover(drone, altitude=2.5, offset_north=0, offset_east=1.0):
    """
    Take off and hover at specified altitude and offset position
    """
    print("-- Taking off and moving to hover position")
    
    # First, regular takeoff to get off the ground
    await drone.action.takeoff()
    await asyncio.sleep(5)  # Wait for initial takeoff
    
    # Initialize offboard mode with current position
    print("-- Initializing offboard mode")
    try:
        # Send a few setpoints before starting offboard mode
        for _ in range(10):
            await drone.offboard.set_velocity_body(
                VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0)
            )
            await asyncio.sleep(0.1)
            
        # Start offboard mode
        await drone.offboard.start()
        
        # Gradually move to desired position
        print("-- Moving to hover position")
        await drone.offboard.set_velocity_body(
            VelocityBodyYawspeed(0.5, 0.0, 0.0, 0.0)  # Move forward slowly
        )
        await asyncio.sleep(2)  # Move for 2 seconds
        
        # Stop at desired position
        await drone.offboard.set_velocity_body(
            VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0)
        )
        
        # Hold position for stabilization
        print("-- Holding position for stabilization")
        for _ in range(50):  # Hold for 5 seconds
            await drone.offboard.set_velocity_body(
                VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0)
            )
            await asyncio.sleep(0.1)
            
        return True
        
    except OffboardError as error:
        print(f"Offboard mode failed with error: {error}")
        await drone.action.land()
        return False
=======
    def __init__(self):
        calib_path = "./opencv/camera_calibration/"
        self.camera_matrix = np.loadtxt(calib_path + 'cameraMatrix_webcam.txt', delimiter=',')
        self.camera_distortion = np.loadtxt(calib_path + 'cameraDistortion_webcam.txt', delimiter=',')
        
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
        self.parameters = aruco.DetectorParameters()
        self.detector = aruco.ArucoDetector(self.aruco_dict, self.parameters)
        
        self.last_detection_time = None
        self.detection_count = 0
        self.frames_processed = 0

    def detect_marker(self, frame):
        self.frames_processed += 1
        frame_copy = frame.copy()
        gray = cv2.cvtColor(frame_copy, cv2.COLOR_BGR2GRAY)
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
            marker_idx = np.where(ids == id_to_find)[0][0]
            
            ret = aruco.estimatePoseSingleMarkers(
                [corners[marker_idx]], marker_size, 
                self.camera_matrix, self.camera_distortion
            )
            rvec, tvec = ret[0][0, 0, :], ret[1][0, 0, :]
            
            aruco.drawDetectedMarkers(frame_copy, corners)
            cv2.drawFrameAxes(frame_copy, self.camera_matrix, 
                            self.camera_distortion, rvec, tvec, 10)
            
            marker_info['detected'] = True
            marker_info['position'] = tvec
            marker_info['attitude'] = self.get_euler_angles(rvec)
            
            self.draw_debug_info(frame_copy, tvec, marker_info['attitude'])
            
        marker_info['frame'] = frame_copy
        return marker_info

    def get_euler_angles(self, rvec):
        R_ct = np.matrix(cv2.Rodrigues(rvec)[0])
        R_tc = R_ct.T
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
        
        pos_text = f"Marker Position: x={position[0]:.1f} y={position[1]:.1f} z={position[2]:.1f}"
        cv2.putText(frame, pos_text, (10, 30), font, 1, (0, 255, 0), 2)
        
        att_text = f"Marker Attitude: r={math.degrees(attitude[0]):.1f} p={math.degrees(attitude[1]):.1f} y={math.degrees(attitude[2]):.1f}"
        cv2.putText(frame, att_text, (10, 60), font, 1, (0, 255, 0), 2)
        
        detection_rate = (self.detection_count / self.frames_processed * 100) if self.frames_processed > 0 else 0
        stats_text = f"Detection rate: {detection_rate:.1f}% ({self.detection_count}/{self.frames_processed})"
        cv2.putText(frame, stats_text, (10, 90), font, 1, (0, 255, 0), 2)

class DroneController:
    def __init__(self):
        self.drone = System()
        self.takeoff_altitude = -1.0  # 5 meters above ground
        self.loiter_radius = 10.0      # 5 meters radius
        self.loiter_speed = 1.0       # 2 m/s
        self.start_time = None
        
    async def setup(self):
        await self.drone.connect(system_address="udp://:14540")
        
        print("Waiting for drone connection...")
        async for state in self.drone.core.connection_state():
            if state.is_connected:
                print("-- Connected to drone!")
                break

        print("Waiting for global position estimate...")
        async for health in self.drone.telemetry.health():
            if health.is_global_position_ok and health.is_home_position_ok:
                print("-- Global position estimate OK")
                break

        print("-- Arming")
        await self.drone.action.arm()

        await asyncio.sleep(2)
        
        # Start offboard mode
        await self.start_offboard()
        
        # Execute takeoff
        await self.takeoff()
        
    async def start_offboard(self):
        print("-- Starting offboard mode")
        try:
            await self.drone.offboard.set_position_ned(
                PositionNedYaw(0.0, 0.0, 0.0, 0.0))
            await self.drone.offboard.start()
        except OffboardError as error:
            print(f"-- Starting offboard mode failed with error: {error}")
            raise

    async def takeoff(self):
        """Execute takeoff sequence"""
        print(f"-- Taking off to {abs(self.takeoff_altitude)} meters")
        await self.drone.offboard.set_position_ned(
            PositionNedYaw(0.0, 0.0, self.takeoff_altitude, 0.0)
        )
        await asyncio.sleep(5)  # Wait for drone to reach altitude
        self.start_time = asyncio.get_event_loop().time()

    async def execute_loiter_pattern(self):
        """Execute circular loiter pattern"""
        current_time = asyncio.get_event_loop().time() - self.start_time
        angle = (current_time * self.loiter_speed) % (2 * math.pi)
        
        x = self.loiter_radius * math.cos(angle)
        y = self.loiter_radius * math.sin(angle)
        yaw = math.degrees(angle)
        
        await self.drone.offboard.set_position_ned(
            PositionNedYaw(x, y, self.takeoff_altitude, yaw)
        )
>>>>>>> fe249176552b0ab121813534f0273a75ff685664

async def fly_search_pattern(drone, marker_detected, center_height=2.5):
    """
    Fly a gentle search pattern while looking for the marker
    """
    if marker_detected:
        # When marker is detected, maintain stable hover
        await drone.offboard.set_velocity_body(
            VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0)
        )
    else:
        # Small figure-8 pattern for searching
        timestamp = time.time()
        period = 10.0  # seconds for one complete pattern
        scale = 0.5    # meters for pattern size
        
        # Calculate smooth velocities for figure-8 pattern
        t = timestamp % period
        phase = (t / period) * 2 * math.pi
        
        vx = scale * math.cos(phase) * 0.2  # Reduced velocity
        vy = scale * math.sin(2 * phase) * 0.2  # Reduced velocity
        
        await drone.offboard.set_velocity_body(
            VelocityBodyYawspeed(vx, vy, 0.0, 0.0)
        )

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
<<<<<<< HEAD
    
    print("-- Arming")
    await drone.action.arm()
    
    # Take off and move to hover position
    if not await takeoff_and_hover(drone, altitude=2.5, offset_north=0, offset_east=1.0):
        print("Failed to reach hover position")
        return
    
    print("-- Reached hover position, starting marker detection")
    
    # Variables for detection status
    marker_detected = False
    last_detection_time = None
    detection_timeout = 2.0  # seconds
    
    # Main control loop
=======

    # Wait for health check
    print("Waiting for drone to be ready...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("-- Global position estimate OK")
            break

    # Arm the drone
    print("-- Arming")
    await drone.action.arm()

    # Start offboard mode
    print("-- Setting initial setpoint")
    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, 0.0, 0.0))
    
    print("-- Starting offboard mode")
    try:
        await drone.offboard.start()
    except Exception as error:
        print(f"-- Starting offboard mode failed with error: {error}")
        return

    # Take off to 5 meters
    print("-- Taking off to 5 meters")
    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, -1.0, 0.0))
    await asyncio.sleep(5)  # Wait for takeoff
    
    # Main loop
>>>>>>> fe249176552b0ab121813534f0273a75ff685664
    while True:
        if video.frame_available():
            frame = video.frame()
            
            # Process frame and detect marker
            marker_info = detector.detect_marker(frame)
            
<<<<<<< HEAD
            # Update marker detection status
            current_time = time.time()
            if marker_info['detected']:
                marker_detected = True
                last_detection_time = current_time
                print(f"\rMarker detected! Position: {marker_info['position']}", end='')
            elif last_detection_time and (current_time - last_detection_time) > detection_timeout:
                marker_detected = False
                print("\rSearching for marker...", end='')
            
            # Display frame with debug info
            cv2.putText(marker_info['frame'], f"FPS: {video.fps}", 
                       (10, 120), cv2.FONT_HERSHEY_PLAIN, 1, (0, 255, 0), 2)
            
            # Add hover status to display
            status_text = "HOVER STABLE" if marker_detected else "SEARCHING"
            cv2.putText(marker_info['frame'], status_text,
                       (10, 150), cv2.FONT_HERSHEY_PLAIN, 1, 
                       (0, 255, 0) if marker_detected else (0, 165, 255), 2)
            
            cv2.imshow("Drone Camera Feed", marker_info['frame'])
            
            # Fly search pattern
            await fly_search_pattern(drone, marker_detected)
        
        # Check for landing command (press 'l')
        if get_key("l"):
            print("\n-- Landing")
            await drone.action.land()
            break
        
        # Check for exit (press 'q' in video window)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            print("\n-- Landing and exiting")
            await drone.action.land()
            break
        
        await asyncio.sleep(0.1)
    
    cv2.destroyAllWindows()

def get_key(keyName):
    ans = False
    for event in pygame.event.get(): pass
    keyInput = pygame.key.get_pressed()
    myKey = getattr(pygame, 'K_{}'.format(keyName))
    if keyInput[myKey]:
        ans = True
    pygame.display.update()
    return ans

def init_pygame():
    pygame.init()
    pygame.display.set_mode((400, 400))

if __name__ == "__main__":
    init_pygame()
=======
            # Display frame with debug info
            cv2.putText(marker_info['frame'], f"FPS: {video.fps}", 
                       (10, 120), cv2.FONT_HERSHEY_PLAIN, 1, (0, 255, 0), 2)
            cv2.imshow("Drone Camera Feed", marker_info['frame'])
            
            # If marker detected, move towards it
            if marker_info['detected']:
                pos = marker_info['position']
                print(f"\rMarker detected! Position: {pos}")
                
                # Convert marker position to NED coordinates and move
                x = -pos[2]  # Forward is negative Z in camera frame
                y = -pos[0]  # Right is negative X in camera frame
                z = -1.0     # Maintain altitude
                
                await drone.offboard.set_position_ned(PositionNedYaw(x, y, z, 0.0))
                
                # If close enough to marker, land
                if abs(x) < 0.5 and abs(y) < 0.5:  # Within 0.5 meters
                    print("\n-- Landing")
                    await drone.action.land()
                    break
            else:
                print("\rSearching for marker...", end='')
                # Maintain position while searching
                await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, -1.0, 0.0))
            
        # Check for exit
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        
        await asyncio.sleep(0.01)  # Small sleep to prevent CPU overload
    
    cv2.destroyAllWindows()

if __name__ == "__main__":
>>>>>>> fe249176552b0ab121813534f0273a75ff685664
    loop = asyncio.get_event_loop()
    loop.run_until_complete(main())