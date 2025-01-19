import asyncio
from mavsdk import System
from mavsdk.offboard import VelocityNedYaw, PositionNedYaw, OffboardError
import cv2
import cv2.aruco as aruco
import numpy as np
import math
from datetime import datetime
from Video import Video

class ArucoDetector:
    def __init__(self):
        # Initialize ArUco detector with multiple dictionaries for better detection
        self.dictionaries = [
            cv2.aruco.DICT_6X6_250,  # Primary dictionary
            cv2.aruco.DICT_5X5_250,  # Fallback dictionaries
            cv2.aruco.DICT_4X4_250
        ]
        self.detectors = [
            cv2.aruco.ArucoDetector(
                cv2.aruco.getPredefinedDictionary(dict_type),
                cv2.aruco.DetectorParameters()
            ) for dict_type in self.dictionaries
        ]
        self.marker_size = 10  # cm
        self.last_detection_time = None
        self.detection_count = 0

    def detect_marker(self, frame):
        if frame is None:
            return None

        # Create a copy of the frame that we can modify
        frame_copy = frame.copy()
        gray = cv2.cvtColor(frame_copy, cv2.COLOR_BGR2GRAY)
        
        # Try each dictionary until we find a marker
        for i, detector in enumerate(self.detectors):
            corners, ids, _ = detector.detectMarkers(gray)
            if ids is not None:
                # Draw the detected markers on the copy
                cv2.aruco.drawDetectedMarkers(frame_copy, corners, ids)
                
                # Get the center of the marker
                marker = corners[0][0]
                center_x = int((marker[0][0] + marker[2][0]) / 2)
                center_y = int((marker[0][1] + marker[2][1]) / 2)
                
                # Calculate marker area for distance estimation
                area = abs((marker[2][0] - marker[0][0]) * (marker[2][1] - marker[0][1]))
                
                self.detection_count += 1
                current_time = datetime.now()
                
                # Print detection info
                print(f"\nMarker Detected!")
                print(f"Dictionary: {self.dictionaries[i]}")
                print(f"Marker IDs: {ids}")
                print(f"Position: ({center_x}, {center_y})")
                print(f"Area: {area}")
                
                return {
                    'corners': corners,
                    'ids': ids,
                    'center': (center_x, center_y),
                    'area': area,
                    'frame': frame_copy
                }
        
        print("\rSearching for markers...", end='', flush=True)
        return None

class DroneController:
    def __init__(self):
        self.drone = None
        self.takeoff_altitude = -2.0  # meters
        self.target_size = 1000  # Reduced from 15000 to be more reasonable
        self.size_threshold = 100  # Reduced from 1000 to match new target size
        self.Kp_xy = 2.0  # Increased from 0.4
        self.Kd_xy = 0.5  # Increased from 0.2
        self.last_error_x = 0
        self.last_error_y = 0
        self.min_velocity = 0.2  # Add minimum velocity threshold
    async def get_flight_mode(self):
        async for flight_mode in self.drone.telemetry.flight_mode():
            return flight_mode

    async def connect(self, connection_string="udp://:14540"):
        self.drone = System()
        await self.drone.connect(system_address=connection_string)
        
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

    async def start_mission(self):
        print("-- Arming")
        await self.drone.action.arm()
        
        print("-- Starting offboard mode")
        try:
            await self.drone.offboard.set_position_ned(
                PositionNedYaw(0.0, 0.0, 0.0, 0.0))
            await self.drone.offboard.start()
        except OffboardError as error:
            print(f"-- Starting offboard mode failed: {error}")
            return False

        print(f"-- Taking off to {abs(self.takeoff_altitude)} meters")
        await self.drone.offboard.set_position_ned(
            PositionNedYaw(0.0, 0.0, self.takeoff_altitude, 0.0)
        )
        await asyncio.sleep(5)  # Wait for takeoff
        return True

    async def move_to_marker(self, marker_info, frame_width, frame_height):
        # Add flight mode check
        flight_mode = await self.get_flight_mode()
        print(f"Current flight mode: {flight_mode}")
        
        if not marker_info:
            await self.drone.offboard.set_position_ned(
                PositionNedYaw(0.0, 0.0, self.takeoff_altitude, 0.0)
            )
            return False

        center_x, center_y = marker_info['center']
        area = marker_info['area']

        # Calculate errors
        error_x = center_x - frame_width / 2
        error_y = center_y - frame_height / 2
        error_z = area - self.target_size

        # Calculate control inputs with PD controller - removed division by 100
        vel_x = -(self.Kp_xy * error_x + self.Kd_xy * (error_x - self.last_error_x)) / 1000
        vel_y = (self.Kp_xy * error_y + self.Kd_xy * (error_y - self.last_error_y)) / 1000
        vel_z = 0.2 if abs(error_z) > self.size_threshold else 0

        # Apply minimum velocity threshold
        if abs(vel_x) < self.min_velocity and vel_x != 0:
            vel_x = self.min_velocity * (vel_x / abs(vel_x))
        if abs(vel_y) < self.min_velocity and vel_y != 0:
            vel_y = self.min_velocity * (vel_y / abs(vel_y))

        # Update last errors
        self.last_error_x = error_x
        self.last_error_y = error_y

        # Get current position for debugging
        # position = await self.drone.telemetry.position_velocity_ned()
        # print("\nMovement Debug:")
        # print(f"Current position: N: {position.position.north_m}m, E: {position.position.east_m}m, D: {position.position.down_m}m")
        # print(f"Errors (x,y,z): ({error_x:.2f}, {error_y:.2f}, {error_z:.2f})")
        # print(f"Velocities (x,y,z): ({vel_x:.2f}, {vel_y:.2f}, {vel_z:.2f})")

        # Send velocity command - simplified direction mapping
        await self.drone.offboard.set_velocity_ned(
            VelocityNedYaw(vel_x, vel_y, 
                          vel_z if area < self.target_size else -vel_z, 0.0)
        )

        # Check if we're centered and at the right height for landing
        if abs(error_x) < 30 and abs(error_y) < 30 and abs(error_z) < self.size_threshold:
            print("\nReady for landing!")
            return True
        return False

    async def land(self):
        print("-- Landing")
        await self.drone.offboard.stop()
        await self.drone.action.land()

async def main():
    # Initialize video and detector
    video = Video(port=5600)  # Make sure port matches your setup
    detector = ArucoDetector()
    drone_controller = DroneController()
    
    # Connect and start mission
    await drone_controller.connect()
    if not await drone_controller.start_mission():
        return

    print("-- Starting marker search pattern")
    frame_count = 0
    
    while True:
        if video.frame_available():
            frame = video.frame()
            frame_count += 1
            
            if frame_count % 30 == 0:  # Print basic info every 30 frames
                if hasattr(video, 'fps'):
                    print(f"Current FPS: {video.fps}")
            
            # Create a copy of the frame for display
            display_frame = frame.copy()
            
            # Detect marker
            marker_info = detector.detect_marker(frame)
            
            # # Display frame with debug info
            # if marker_info:
            #     cv2.putText(marker_info['frame'], f"Marker detected at {marker_info['center']}", 
            #                (10, 30), cv2.FONT_HERSHEY_PLAIN, 1, (0, 255, 0), 2)
            #     cv2.putText(marker_info['frame'], f"Area: {marker_info['area']}", 
            #                (10, 60), cv2.FONT_HERSHEY_PLAIN, 1, (0, 255, 0), 2)
            #     display_frame = marker_info['frame']  # Use the annotated frame
            
            cv2.imshow("Drone Camera Feed", display_frame)
            
            # Move towards marker if detected
            if marker_info:
                ready_to_land = await drone_controller.move_to_marker(
                    marker_info, frame.shape[1], frame.shape[0]
                )
                
                if ready_to_land:
                    await drone_controller.land()
                    break
            else:
                # Maintain position while searching
                await drone_controller.drone.offboard.set_position_ned(
                    PositionNedYaw(0.0, 0.0, drone_controller.takeoff_altitude, 0.0)
                )
            
        # Check for exit
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        
        await asyncio.sleep(0.01)
    
    cv2.destroyAllWindows()

if __name__ == "__main__":
    asyncio.run(main())