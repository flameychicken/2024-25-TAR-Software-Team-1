import asyncio
from mavsdk import System
from mavsdk.offboard import PositionNedYaw
import numpy as np
import cv2
import cv2.aruco as aruco
from gi.repository import Gst

# Define Aruco ID and marker size
id_to_find = 72
marker_size = 25  # cm

# Default camera matrix and distortion coefficients for simulation
camera_matrix = np.array([[800, 0, 640],   # fx, 0, cx
                          [0, 800, 360],   # 0, fy, cy
                          [0, 0, 1]], dtype=np.float32)

camera_distortion = np.array([0, 0, 0, 0], dtype=np.float32)

class Video:
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

    async def setup(self, port='udp://:14540'):
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
        initial_setpoint = PositionNedYaw(0.0, 0.0, -3.0, 0.0)  # Set to hover at 3 meters altitude
        await self.drone.offboard.set_position_ned(initial_setpoint)
        await self.drone.offboard.start()

        # Confirm stable hovering at target altitude before tilting camera
        await self.wait_until_hovered(-3.0)
        await self.set_camera_tilt_down()

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
        await self.drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, -5.0, 0.0))  # Hover at -5 meters
        await asyncio.sleep(2)  # Allow time to stabilize

        print("-- Tilting camera downward by adjusting yaw")
        for angle in range(0, -90, -10):  # Adjusting yaw gradually
            await self.drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, -5.0, angle))
            print(f"-- Adjusting yaw to {angle} degrees")
            await asyncio.sleep(0.5)  # Pause briefly for each yaw adjustment

        print("-- Tilt adjustment complete, ready to proceed.")

    async def search_and_land(self):
        """Continuously search for the box and land when detected."""
        while True:
            if not self.video.frame_available():
                await asyncio.sleep(0.1)
                continue

            frame = self.video.frame()
            if frame is not None:
                # Detect the box and print "1" if detected
                if self.detect_box(frame):
                    print("1")  # Debugging print statement for detecting box
                    print("-- Box detected, initiating landing.")
                    await self.drone.action.land()
                    break  # Land the drone when the box is detected

            await asyncio.sleep(0.1)

    def detect_box(self, frame):
        """Detect a box in the frame using color thresholding."""
        # Convert the frame to HSV (Hue, Saturation, Value) space for easier color-based detection
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Define the range of color values for the box (e.g., a green box)
        lower_bound = np.array([30, 50, 50])  # Lower bound of color in HSV (for green)
        upper_bound = np.array([90, 255, 255])  # Upper bound of color in HSV (for green)

        # Create a mask using the defined color range
        mask = cv2.inRange(hsv, lower_bound, upper_bound)

        # Find contours in the mask
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # If contours are detected, assume a box is present
        if contours:
            # Optionally draw the contours on the frame for visualization
            cv2.drawContours(frame, contours, -1, (0, 255, 0), 3)
            return True  # Box detected

        return False  # No box detected

    def detect_aruco_marker(self, frame):
        aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
        parameters = aruco.DetectorParameters()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        detector = aruco.ArucoDetector(aruco_dict, parameters)
        corners, ids, _ = detector.detectMarkers(gray)

        if ids is not None:
            print(f"Detected IDs: {ids}")  # Print the detected IDs
            if id_to_find in ids:
                aruco.drawDetectedMarkers(frame, corners)
                return True
        return False

async def main():
    controller = DroneController()
    await controller.setup()
    await controller.search_and_land()

if __name__ == "__main__":
    asyncio.run(main())
