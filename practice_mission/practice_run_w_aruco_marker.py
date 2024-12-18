import asyncio
from mavsdk import System
from mavsdk.offboard import PositionNedYaw, OffboardError
import cv2
import cv2.aruco as aruco
import numpy as np
from gi.repository import Gst

# Define Aruco ID and marker size
id_to_find = 72
marker_size = 10  # cm

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
        initial_setpoint = PositionNedYaw(0.0, 0.0, -3.0, 0.0)
        await self.drone.offboard.set_position_ned(initial_setpoint)
        await self.drone.offboard.start()

    async def search_and_land(self):
        while True:
            if not self.video.frame_available():
                await asyncio.sleep(0.1)
                continue

            frame = self.video.frame()
            if frame is not None and self.detect_aruco_marker(frame):
                await self.move_toward_marker(frame)
                print("-- Landing sequence complete.")
                break

            await asyncio.sleep(0.1)

    async def move_toward_marker(self, frame):
        # Calculate distance and movement towards the marker
        rvec, tvec = self.get_marker_position(frame)

        if tvec is not None:
            while tvec[0][0][2] > 0.2:  # Continue until close to marker
                await self.drone.offboard.set_position_ned(
                    PositionNedYaw(tvec[0][0][0], tvec[0][0][1], -3.0, 0.0)
                )
                print(f"-- Approaching marker at x={tvec[0][0][0]}, y={tvec[0][0][1]}, z={tvec[0][0][2]}")
                await asyncio.sleep(0.5)

            print("-- Marker detected, initiating landing.")
            await self.drone.action.land()

    def detect_aruco_marker(self, frame):
        calib_path = "./opencv/camera_calibration/"
        camera_matrix = np.loadtxt(calib_path + 'cameraMatrix_webcam.txt', delimiter=',')
        camera_distortion = np.loadtxt(calib_path + 'cameraDistortion_webcam.txt', delimiter=',')

        aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
        parameters = aruco.DetectorParameters()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        detector = aruco.ArucoDetector(aruco_dict, parameters)
        corners, ids, _ = detector.detectMarkers(gray)

        if ids is not None and id_to_find in ids:
            aruco.drawDetectedMarkers(frame, corners)
            return True
        return False

    def get_marker_position(self, frame):
        calib_path = "./opencv/camera_calibration/"
        camera_matrix = np.loadtxt(calib_path + 'cameraMatrix_webcam.txt', delimiter=',')
        camera_distortion = np.loadtxt(calib_path + 'cameraDistortion_webcam.txt', delimiter=',')

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