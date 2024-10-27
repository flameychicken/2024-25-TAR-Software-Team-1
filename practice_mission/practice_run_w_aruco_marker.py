import asyncio
from mavsdk import System
from mavsdk.offboard import PositionNedYaw, OffboardError, VelocityBodyYawspeed
import pygame
import cv2
import cv2.aruco as aruco
import numpy as np
import sys, time, math
from gi.repository import Gst

# Define Aruco ID and marker size for detection
id_to_find = 72
marker_size = 10  # in cm

# Video class for capturing video stream
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
            f'udpsrc port={self.port} ! application/x-rtp, payload=96 ! rtph264depay ! h264parse ! avdec_h264',
            '! decodebin ! videoconvert ! video/x-raw,format=(string)BGR ! videoconvert',
            '! appsink emit-signals=true sync=false max-buffers=2 drop=true'
        ])
        self.video_sink.connect('new-sample', self.callback)

    def callback(self, sink):
        sample = sink.emit('pull-sample')
        new_frame = self.gst_to_opencv(sample)
        self._frame = new_frame
        return Gst.FlowReturn.OK

class DroneController:
    def __init__(self):
        self.drone = System()
        self.video = Video()  # Initialize video stream

    async def practice_run(self, port='udp://:14540'):
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
        await asyncio.sleep(0.5)

        # Start detecting Aruco markers in parallel
        print("-- Starting offboard")
        try:
            await self.drone.offboard.start()
            print("-- Offboard mode started successfully")
        except OffboardError as error:
            print(f"Offboard start failed with error code: {error._result.result}")
            await self.drone.action.disarm()
            return

        # Start both the flight and detection tasks
        await asyncio.gather(self.fly_square(2), self.detect_flag())

    async def fly_square(self, n: int):
        square_path = [
            PositionNedYaw(5.0, 0.0, -1.0, 0.0),
            PositionNedYaw(5.0, 5.0, -1.0, 90.0),
            PositionNedYaw(0.0, 5.0, -1.0, 180.0),
            PositionNedYaw(0.0, 0.0, -1.0, 270.0)
        ]

        for i in range(n):
            print(f"-- Starting square loop {i + 1}")
            for j, position in enumerate(square_path):
                print(f"-- Moving to waypoint {j + 1} of loop {i + 1}")
                await self.drone.offboard.set_position_ned(position)
                await asyncio.sleep(5)

        print("-- Landing")
        await self.drone.action.land()

    async def detect_flag(self):
        while True:
            if not self.video.frame_available():
                await asyncio.sleep(0.1)
                continue

            frame = self.video.frame()
            if frame is None:
                continue

            if self.detect_aruco_marker(frame):
                print("Flag detected!")

            await asyncio.sleep(0.5)

    def detect_aruco_marker(self, frame):
        # Load camera calibration
        calib_path = "./opencv/camera_calibration/"
        camera_matrix = np.loadtxt(calib_path + 'cameraMatrix_webcam.txt', delimiter=',')
        camera_distortion = np.loadtxt(calib_path + 'cameraDistortion_webcam.txt', delimiter=',')

        # Define Aruco dictionary
        aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
        parameters = aruco.DetectorParameters()

        # Convert to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        detector = aruco.ArucoDetector(aruco_dict, parameters)
        corners, ids, _ = detector.detectMarkers(gray)

        if ids is not None and id_to_find in ids:
            index = np.where(ids == id_to_find)[0][0]
            rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners[index], marker_size, camera_matrix, camera_distortion)

            aruco.drawDetectedMarkers(frame, corners)
            cv2.drawFrameAxes(frame, camera_matrix, camera_distortion, rvec, tvec, 10)

            # Get position in camera frame
            str_position = f"Marker Position x={tvec[0][0]:.2f} y={tvec[0][1]:.2f} z={tvec[0][2]:.2f}"
            print(str_position)

            return True
        return False


if __name__ == "__main__":
    controller = DroneController()
    loop = asyncio.get_event_loop()
    loop.run_until_complete(controller.practice_run())
