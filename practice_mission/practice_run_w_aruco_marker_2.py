import asyncio
from mavsdk import System
from mavsdk.offboard import PositionNedYaw
import numpy as np
import cv2
import cv2.aruco as aruco
from gi.repository import Gst
import math

# Define Aruco ID and marker size
id_to_find = 72
marker_size = 25  # cm

# Default camera matrix and distortion coefficients for simulation
camera_matrix = np.array([[800, 0, 640],   # fx, 0, cx
                         [0, 800, 360],    # 0, fy, cy
                         [0, 0, 1]], dtype=np.float32)

camera_distortion = np.array([0, 0, 0, 0], dtype=np.float32)

class Video:
    # [Previous Video class implementation remains the same]
    def __init__(self, port=5600):
        Gst.init(None)
        self.port = port
        self._frame = None
        self.run()

    def start_gst(self, config=None):
        if not config:
            config = [
                'videotestsrc ! decodebin',
                '! videoconvert ! video/x-raw,format=(string)BGR ! videoconvert',
                '! appsink'
            ]
        command = ' '.join(config)
        self.video_pipe = Gst.parse_launch(command)
        self.video_pipe.set_state(Gst.State.PLAYING)
        self.video_sink = self.video_pipe.get_by_name('appsink0')

    @staticmethod
    def gst_to_opencv(sample):
        buf = sample.get_buffer()
        caps = sample.get_caps()
        array = np.ndarray(
            (caps.get_structure(0).get_value('height'), caps.get_structure(0).get_value('width'), 3),
            buffer=buf.extract_dup(0, buf.get_size()), dtype=np.uint8)
        return array

    def frame(self):
        return self._frame

    def frame_available(self):
        return self._frame is not None

    def run(self):
        self.start_gst([
            f'udpsrc port={self.port} ! application/x-rtp, payload=96 ! rtph264depay ! h264parse ! avdec_h264',
            '! decodebin ! videoconvert ! video/x-raw,format=(string)BGR ! videoconvert',
            '! appsink emit-signals=true sync=false max-buffers=2 drop=true'
        ])
        self.video_sink.connect('new-sample', self.callback)

    def callback(self, sink):
        sample = sink.emit('pull-sample')
        self._frame = self.gst_to_opencv(sample)
        return Gst.FlowReturn.OK

class DroneController:
    def __init__(self):
        self.drone = System()
        self.video = Video()
        self.search_radius = 10.0  # meters
        self.search_altitude = -5.0  # meters
        self.marker_detected = False
        self.current_phase = 0
        self.spiral_angle = 0
        self.spiral_radius = 2.0

    async def setup(self, port='udp://:14540'):
        # [Previous setup implementation remains the same]
        await self.drone.connect(system_address=port)
        print("Waiting for connection...")
        async for state in self.drone.core.connection_state():
            if state.is_connected:
                print("-- connection successful")
                break

        print("Waiting for global position...")
        async for health in self.drone.telemetry.health():
            if health.is_global_position_ok and health.is_home_position_ok:
                print("-- Global position estimate OK")
                break

        await asyncio.sleep(1)
        print("-- Arming")
        await self.drone.action.arm()

        print("-- Starting offboard mode")
        initial_setpoint = PositionNedYaw(0.0, 0.0, self.search_altitude, 0.0)
        await self.drone.offboard.set_position_ned(initial_setpoint)
        await self.drone.offboard.start()

        await self.wait_until_hovered(self.search_altitude)
        await self.set_camera_tilt_down()

    async def execute_search_pattern(self):
        """Execute an expanding spiral search pattern until marker is detected"""
        while not self.marker_detected:
            # Calculate next position in spiral pattern
            x = self.spiral_radius * math.cos(self.spiral_angle)
            y = self.spiral_radius * math.sin(self.spiral_angle)
            
            # Move to next position
            await self.drone.offboard.set_position_ned(
                PositionNedYaw(x, y, self.search_altitude, 0.0)
            )
            
            # Check for marker
            if self.video.frame_available():
                frame = self.video.frame()
                if frame is not None and self.detect_aruco_marker(frame):
                    self.marker_detected = True
                    print("-- Marker detected! Switching to precision approach.")
                    break
            
            # Update spiral parameters
            self.spiral_angle += 0.1
            self.spiral_radius += 0.02
            
            # Safety boundary check
            if self.spiral_radius > self.search_radius:
                self.spiral_radius = 2.0
                self.spiral_angle = 0
            
            await asyncio.sleep(0.1)

    async def precision_approach(self):
        """Execute precision approach once marker is detected"""
        print("-- Beginning precision approach")
        while True:
            if not self.video.frame_available():
                await asyncio.sleep(0.1)
                continue

            frame = self.video.frame()
            if frame is None:
                continue

            rvec, tvec = self.get_marker_position(frame)
            if tvec is not None:
                # Convert marker position to drone coordinates
                x_offset = tvec[0][0][0]
                y_offset = tvec[0][0][1]
                z_offset = tvec[0][0][2]

                # Gradually decrease altitude while maintaining position over marker
                current_alt = self.search_altitude
                while current_alt < -0.5:  # Stop at 0.5m above ground
                    await self.drone.offboard.set_position_ned(
                        PositionNedYaw(x_offset, y_offset, current_alt, 0.0)
                    )
                    current_alt += 0.2  # Move down slowly
                    await asyncio.sleep(0.5)

                print("-- In position for landing")
                await self.drone.action.land()
                break
            
            await asyncio.sleep(0.1)

    async def search_and_land(self):
        """Main control loop combining search pattern and precision approach"""
        try:
            # Start with search pattern
            await self.execute_search_pattern()
            
            # Once marker is detected, switch to precision approach
            if self.marker_detected:
                await self.precision_approach()
        except Exception as e:
            print(f"Error during mission: {e}")
            await self.drone.action.return_to_launch()

    # [Previous helper methods remain the same]
    async def wait_until_hovered(self, target_altitude, tolerance=0.2):
        """Wait until the drone stabilizes at the target altitude."""
        print("-- Waiting for stable hover")
        async for position in self.drone.telemetry.position():
            if abs(position.relative_altitude_m - abs(target_altitude)) <= tolerance:
                print(f"-- Hovered at {target_altitude} meters")
                break
            await asyncio.sleep(0.1)

    async def set_camera_tilt_down(self):
        print("-- Ascending to a stable altitude for better downward view")
        await self.drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, -5.0, 0.0))
        await asyncio.sleep(2)

        print("-- Tilting camera downward by adjusting yaw")
        for angle in range(0, -90, -10):
            await self.drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, -5.0, angle))
            print(f"-- Adjusting yaw to {angle} degrees")
            await asyncio.sleep(0.5)

        print("-- Tilt adjustment complete, ready to proceed.")

    def detect_aruco_marker(self, frame):
        aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
        parameters = aruco.DetectorParameters()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        detector = aruco.ArucoDetector(aruco_dict, parameters)
        corners, ids, _ = detector.detectMarkers(gray)

        if ids is not None:
            print(f"Detected IDs: {ids}")
            if id_to_find in ids:
                aruco.drawDetectedMarkers(frame, corners)
                return True
        return False

    def get_marker_position(self, frame):
        aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
        parameters = aruco.DetectorParameters()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        detector = aruco.ArucoDetector(aruco_dict, parameters)
        corners, ids, _ = detector.detectMarkers(gray)

        if ids is not None and id_to_find in ids:
            index = np.where(ids == id_to_find)[0][0]
            rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners[index], marker_size, camera_matrix, camera_distortion)
            return rvec, tvec
        return None, None

async def main():
    controller = DroneController()
    await controller.setup()
    await controller.search_and_land()

if __name__ == "__main__":
    asyncio.run(main())