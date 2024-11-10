import asyncio
from mavsdk import System
from mavsdk.offboard import PositionNedYaw, Attitude
import numpy as np
import cv2
from gi.repository import Gst

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
        print("-- Tilting camera downward by adjusting pitch")
        attitude = Attitude(pitch_deg=90, roll_deg=0, yaw_deg=0, thrust=0.5)
        await self.drone.offboard.set_attitude(attitude)
        await asyncio.sleep(2)
        print("-- Tilt adjustment complete, ready to proceed.")

    async def search_and_land(self):
        """Continuously search for any detectable object and land when detected."""
        while True:
            if not self.video.frame_available():
                await asyncio.sleep(0.1)
                continue

            frame = self.video.frame()
            if frame is not None and self.detect_object_in_frame(frame):
                print("-- Object detected, initiating landing.")
                await self.drone.action.land()
                break

            await asyncio.sleep(0.1)

    def detect_object_in_frame(self, frame):
        # Convert frame to grayscale (or other processing as needed)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Simple example using color range to detect an object
        lower_bound = np.array([30, 150, 50])  # Adjust based on color of target object
        upper_bound = np.array([255, 255, 180])
        mask = cv2.inRange(frame, lower_bound, upper_bound)

        # Detect contours to identify objects
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            print("-- Object detected, contours found.")
            return True
        return False


async def main():
    controller = DroneController()
    await controller.setup()
    await controller.search_and_land()

if __name__ == "__main__":
    asyncio.run(main())
