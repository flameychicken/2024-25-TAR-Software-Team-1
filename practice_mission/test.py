import asyncio
from mavsdk import System
from mavsdk.offboard import VelocityBodyYawspeed
import pygame
from mavsdk.action import OrbitYawBehavior

def init():
    print("Initializing pygame...")
    pygame.init()
    pygame.display.set_mode((400, 400))
    print("Pygame initialized.")

async def main():
    print("Connecting to drone...")
    drone = System()
    await drone.connect(system_address="udp://:14445")

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
    await asyncio.sleep(5)


    print('Do orbit at 10m height from the ground')
    await drone.action.do_orbit(
        radius_m=10,
        velocity_ms=2,
        yaw_behavior=OrbitYawBehavior.HOLD_FRONT_TO_CIRCLE_CENTER,
        latitude_deg=float("nan"),
        longitude_deg=float("nan"),
        absolute_altitude_m=float("nan")
    )
    video = cv2.VideoCapture(1)
    
    await asyncio.sleep(60)

if __name__ == "__main__":
    init()
    loop = asyncio.get_event_loop()
    loop.run_until_complete(main())
