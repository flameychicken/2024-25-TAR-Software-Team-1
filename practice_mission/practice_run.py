import asyncio
from mavsdk import System
from mavsdk.offboard import VelocityBodyYawspeed

async def orbit(drone, orbit_height, yaw_behavior):
    print('Starting orbit at {}m height from the ground'.format(orbit_height))
    await drone.action.do_orbit(
        radius_m=10,  # Orbit radius in meters
        velocity_ms=2,  # Orbiting speed in meters per second
        yaw_behavior=yaw_behavior,  # Yaw behavior (e.g., 'yaw_behavior.YAW_LOCKED')
        latitude_deg=47.398036222362471,  # Replace with your target latitude
        longitude_deg=8.5450146439425509,  # Replace with your target longitude
        absolute_altitude_m=orbit_height  # Altitude for the orbit
    )
    await asyncio.sleep(60)  # Orbit for 60 seconds

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

    # Start the orbiting function
    await orbit(drone, orbit_height=10, yaw_behavior="yaw_behavior.YAW_LOCKED")

    print("-- Landing")
    await drone.action.land()

if __name__ == "__main__":
    loop = asyncio.get_event_loop()
    loop.run_until_complete(main())
