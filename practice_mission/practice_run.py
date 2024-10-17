import asyncio
from mavsdk import System
from mavsdk.offboard import VelocityBodyYawspeed

async def main():
    print("Connecting to drone...")
    drone = System()
    await drone.connect(system_address="udp://:14540")

    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("-- Connected to drone!")
            break

    print("-- Arming")
    await drone.action.arm()

    print("-- Taking off")
    await drone.action.takeoff()

    # Wait for the drone to reach a stable altitude
    await asyncio.sleep(5)

    print("-- Setting offboard mode")
    await drone.offboard.start()

    # Circulate for 60 seconds
    start_time = asyncio.get_event_loop().time()
    while asyncio.get_event_loop().time() - start_time < 60:
        # Set a velocity for hovering in place (0 forward, 0 right, 0 down)
        velocity = VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0)
        await drone.offboard.set_velocity_body(velocity)

        await asyncio.sleep(0.1)  # Update frequency

    print("-- Landing")
    await drone.action.land()

'''
Loitering Function
'''
async def orbit(drone, orbit_height, yaw_behavior):
    print('Do orbit at 10m height from the ground')
    await drone.action.do_orbit(
        radius_m=10,
        velocity_ms=2,
        yaw_behavior=yaw_behavior,
        latitude_deg=float("nan"),
        longitude_deg=float("nan"),
        absolute_altitude_m=float("nan")
    )
    await asyncio.sleep(60)

if __name__ == "__main__":
    loop = asyncio.get_event_loop()
    loop.run_until_complete(main())
