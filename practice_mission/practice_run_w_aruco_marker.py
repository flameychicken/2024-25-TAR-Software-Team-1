import asyncio
from mavsdk import System
from mavsdk.offboard import PositionNedYaw, OffboardError
import cv2
import cv2.aruco as aruco
import numpy as np
import time

# Define Aruco ID and marker size for detection
id_to_find = 72
marker_size = 10  # in cm

class DroneController:
    def __init__(self):
        self.drone = System()
        self.video = Video(port=5600)  # Initialize video stream

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

        # Start both the flight and detection tasks
        print("-- Starting offboard")
        try:
            await self.drone.offboard.start()
            print("-- Offboard mode started successfully")
        except OffboardError as error:
            print(f"Offboard start failed with error code: {error._result.result}")
            await self.drone.action.disarm()
            return

        await asyncio.gather(self.fly_square(2), self.detect_and_land())

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

        print("-- Completed square path")

    async def detect_and_land(self):
        while True:
            if not self.video.frame_available():
                await asyncio.sleep(0.1)
                continue

            frame = self.video.frame()
            if frame is None:
                continue

            # If marker detected, initiate landing
            if self.detect_aruco_marker(frame):
                print("Landing pad detected, initiating landing!")
                await self.drone.action.land()
                break  # Exit detection loop after initiating landing

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

            # Draw detected marker and pose axes
            aruco.drawDetectedMarkers(frame, corners)
            cv2.drawFrameAxes(frame, camera_matrix, camera_distortion, rvec, tvec, 10)

            # Print marker's position relative to the drone
            str_position = f"Marker Position x={tvec[0][0]:.2f} y={tvec[0][1]:.2f} z={tvec[0][2]:.2f}"
            print(str_position)

            return True  # Marker detected
        return False  # Marker not detected


if __name__ == "__main__":
    controller = DroneController()
    loop = asyncio.get_event_loop()
    loop.run_until_complete(controller.practice_run())
