import asyncio
from mavsdk import System
from mavsdk.offboard import VelocityBodyYawspeed
import pygame
from mavsdk.action import OrbitYawBehavior

#import cv2
#import cvlib as cv
#from cvlib.object_detection import draw_bbox

def init():
    print("Initializing pygame...")
    pygame.init()
    pygame.display.set_mode((400, 400))
    print("Pygame initialized.")

"""def human_detection(video):
    ret, frame = video.read()
    # Bounding box.
    # the cvlib library has learned some basic objects using object learning
    # usually it takes around 800 images for it to learn what a phone is.
    bbox, label, conf = cv.detect_common_objects(frame)
    
    output_image = draw_bbox(frame, bbox, label, conf)
    
    cv2.imshow("Detection", output_image)
    
    if "person" in label:
        return True
    else:
        return False"""

async def print_position(drone):
    async for position in drone.telemetry.position():
        print(position)

async def main():
    print("Connecting to drone...")
    drone = System()
    await drone.connect(system_address="udp://:14540")

    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("-- Connected to drone!")
            break

    # print("Waiting for global position estimate...")
    # async for health in drone.telemetry.health():
    #     if health.is_global_position_ok and health.is_home_position_ok:
    #         print("-- Global position estimate OK")
    #         break

    print("-- Arming")
    await drone.action.arm()

    print("-- Taking off")
    await drone.action.takeoff()

    # Wait for the drone to reach a stable altitude
    await asyncio.sleep(10)
    asyncio.ensure_future(print_position(drone))
    print('Do orbit at 10m height from the ground')
    await drone.action.do_orbit(
        radius_m=2.0,
        velocity_ms=2.0,
        yaw_behavior=OrbitYawBehavior.HOLD_FRONT_TO_CIRCLE_CENTER,
        latitude_deg=47.398036222362471,
        longitude_deg=8.5450146439425509,
        absolute_altitude_m=float("NaN")
    )
    """
    while True:
        video = cv2.VideoCapture(1)
        if human_detection(video):
            print('human detected')
            break  
    """  
    await asyncio.sleep(60)
    print("done")

if __name__ == "__main__":
    #init()
    loop = asyncio.get_event_loop()
    loop.run_until_complete(main())
