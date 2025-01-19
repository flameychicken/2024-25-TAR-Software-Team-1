import asyncio
from mavsdk import System
from mavsdk.offboard import (
    VelocityBodyYawspeed,
    PositionNedYaw,
    OffboardError
)
import cv2
import cv2.aruco as aruco
import numpy as np
import time
import math
from datetime import datetime

class ArucoDetector:
    def __init__(self):
        # Initialize ArUco detector with multiple dictionaries
        self.dictionaries = [
            cv2.aruco.DICT_4X4_250,
            cv2.aruco.DICT_5X5_250,
            cv2.aruco.DICT_6X6_250,
            cv2.aruco.DICT_7X7_250,
            cv2.aruco.DICT_ARUCO_ORIGINAL
        ]
        self.detectors = [
            cv2.aruco.ArucoDetector(
                cv2.aruco.getPredefinedDictionary(dict_type),
                cv2.aruco.DetectorParameters()
            )
            for dict_type in self.dictionaries
        ]
        
        # Detection statistics
        self.detection_count = 0
        self.frames_processed = 0
        self.last_detection_time = None

    def detect_marker(self, frame):
        self.frames_processed += 1
        frame_copy = frame.copy()
        img_gray = cv2.cvtColor(frame_copy, cv2.COLOR_BGR2GRAY)
        
        for detector in self.detectors:
            bbox, ids, _ = detector.detectMarkers(img_gray)
            
            if ids is not None:
                self.detection_count += 1
                self.last_detection_time = datetime.now()
                
                # Draw markers
                cv2.aruco.drawDetectedMarkers(frame_copy, bbox, ids)
                
                # Calculate marker center and area
                marker = bbox[0][0]
                center_x = int((marker[0][0] + marker[2][0]) / 2)
                center_y = int((marker[0][1] + marker[2][1]) / 2)
                area = abs((marker[2][0] - marker[0][0]) * (marker[2][1] - marker[0][1]))
                
                # Draw debug information
                self.draw_debug_info(frame_copy, center_x, center_y, area)
                
                return {
                    'detected': True,
                    'bbox': bbox,
                    'ids': ids,
                    'center': (center_x, center_y),
                    'area': area,
                    'frame': frame_copy
                }
        
        return {
            'detected': False,
            'bbox': None,
            'ids': None,
            'center': None,
            'area': None,
            'frame': frame_copy
        }

    def draw_debug_info(self, frame, center_x, center_y, area):
        font = cv2.FONT_HERSHEY_PLAIN
        detection_rate = (self.detection_count / self.frames_processed * 100) if self.frames_processed > 0 else 0
        
        cv2.putText(frame, f"Center: ({center_x}, {center_y})", (10, 30), font, 1, (0, 255, 0), 2)
        cv2.putText(frame, f"Area: {area:.0f}", (10, 60), font, 1, (0, 255, 0), 2)
        cv2.putText(frame, f"Detection rate: {detection_rate:.1f}%", (10, 90), font, 1, (0, 255, 0), 2)

class PrecisionLandingController:
    def __init__(self):
        self.Kp_xy = 0.4  # Position proportional gain
        self.Kd_xy = 0.5  # Position derivative gain
        self.Kp_z = 0.3   # Altitude proportional gain
        self.last_error_x = 0
        self.last_error_y = 0
        self.desired_size = 11000  # Target marker size in pixels
        self.size_threshold = 1000
        self.center_threshold = 30  # Pixels from center to consider "centered"

    def calculate_control(self, marker_info, frame_width, frame_height):
        if not marker_info['detected']:
            return 0, 0, 0, False

        center_x, center_y = marker_info['center']
        area = marker_info['area']

        # Calculate errors
        error_x = center_x - frame_width / 2
        error_y = center_y - frame_height / 2
        error_z = area - self.desired_size

        # Calculate control signals with PD controller
        control_x = -(self.Kp_xy * error_x + self.Kd_xy * (error_x - self.last_error_x))
        control_y = self.Kp_xy * error_y + self.Kd_xy * (error_y - self.last_error_y)
        control_z = self.Kp_z if abs(error_z) > self.size_threshold else 0

        # Update last errors
        self.last_error_x = error_x
        self.last_error_y = error_y

        # Check if centered and at correct height
        ready_to_land = (abs(error_x) < self.center_threshold and 
                        abs(error_y) < self.center_threshold and 
                        abs(error_z) < self.size_threshold)

        return (control_x / 100, 
                control_y / 100, 
                control_z if area < self.desired_size else -control_z,
                ready_to_land)

async def land_on_marker(drone, video_source):
    """
    Execute precision landing using ArUco marker detection.
    """
    detector = ArucoDetector()
    controller = PrecisionLandingController()
    w, h = 640, 480
    
    try:
        # Start offboard mode
        print("-- Starting offboard control")
        await drone.offboard.set_position_ned(PositionNedYaw(0, 0, -2, 0))
        await drone.offboard.start()

        while True:
            if video_source.frame_available():
                frame = video_source.frame()
                if frame is None:
                    continue
                
                frame = cv2.resize(frame, (w, h))
                marker_info = detector.detect_marker(frame)
                
                if marker_info['detected']:
                    # Calculate control inputs
                    vx, vy, vz, ready_to_land = controller.calculate_control(
                        marker_info, w, h)
                    
                    if ready_to_land:
                        print("-- Marker centered, initiating landing")
                        await drone.offboard.stop()
                        await drone.action.land()
                        return True
                    
                    # Execute movement
                    await drone.offboard.set_velocity_body(
                        VelocityBodyYawspeed(vx, vy, vz, 0))
                else:
                    # Hover in place if no marker detected
                    await drone.offboard.set_velocity_body(
                        VelocityBodyYawspeed(0, 0, 0, 0))
                
                # Display frame
                cv2.imshow('Precision Landing View', marker_info['frame'])
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

            await asyncio.sleep(0.1)

    except Exception as e:
        print(f"Precision landing error: {e}")
        return False
    finally:
        cv2.destroyAllWindows()

async def main():
    # Initialize drone connection
    drone = System()
    await drone.connect(system_address="udp://:14540")
    
    print("Waiting for drone connection...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("-- Connected to drone!")
            break

    # Wait for drone to be ready
    print("-- Arming drone")
    await drone.action.arm()

    # Initialize video source
    from Video import Video  # Make sure Video class is imported from provided code
    video_source = Video(port=5601)

    # Execute precision landing
    success = await land_on_marker(drone, video_source)
    print(f"Precision landing {'successful' if success else 'failed'}")

if __name__ == "__main__":
    asyncio.run(main())