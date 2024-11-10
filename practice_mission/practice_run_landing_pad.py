import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
from mavsdk import System

# Setup the ROS node and CvBridge
rospy.init_node('camera_listener')
bridge = CvBridge()

# Connect to the PX4 drone via MAVSDK
async def connect_to_drone():
    drone = System()
    await drone.connect(system_address="udp://:14540")  # Connect to PX4 SITL
    print("Connected to the drone")
    return drone

async def takeoff(drone):
    await drone.action.arm()
    await drone.action.takeoff()
    print("Drone taking off...")

async def detect_landing_pad(frame):
    """Detect ArUco marker and land on it"""
    aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
    aruco_params = cv2.aruco.DetectorParameters_create()

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)

    if ids is not None:
        for marker_corners in corners:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            print(f"Marker detected: {ids[0]}")

            # Trigger the landing on the detected marker
            return True  # Trigger landing
    return False

def image_callback(msg):
    """Callback function to handle image feed"""
    # Convert the ROS Image message to an OpenCV image
    frame = bridge.imgmsg_to_cv2(msg, "bgr8")
    print("Received frame")

    # Detect landing pad
    if detect_landing_pad(frame):
        # Land the drone
        rospy.signal_shutdown("Landing pad detected. Landing the drone.")
        # Call your MAVSDK landing function here

    # Display the camera feed with detected markers
    cv2.imshow("Camera Feed", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        rospy.signal_shutdown("Exit key pressed")

def main():
    # Start MAVSDK connection
    loop = asyncio.get_event_loop()
    drone = loop.run_until_complete(connect_to_drone())
    loop.run_until_complete(takeoff(drone))

    # ROS Subscriber for the camera feed
    rospy.Subscriber('/camera/image_raw', Image, image_callback)

    rospy.spin()  # Keeps the script running to receive ROS messages

if __name__ == "__main__":
    main()
