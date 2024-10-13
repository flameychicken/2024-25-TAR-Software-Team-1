import asyncio
from mavsdk import System
from mavsdk.offboard import VelocityBodyYawspeed

async def orbit(drone, orbit_radius, orbit_speed):
    print('Starting orbit...')
    # Set the initial angle and velocity
    angle = 0
    while angle < 360:
        # Convert angle to radians for circular motion
        radians = angle * (3.14159 / 180.0)
        forward = orbit_speed * -1  # Adjust forward motion to go circular
        right = orbit_radius * -1 * (radians % (2 * 3.14159))  # Circular right motion
        
        # Set velocity to create circular motion
        velocity = VelocityBodyYawspeed(forward, right, 0, 0)
        await drone.offboard.set_velocity_body(velocity)

        angle += 5  # Increment angle (degrees)
        await asyncio.sleep(0.1)  # Update frequency

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
    await asyncio.sleep(5)  # Stabilization time

    print("-- Setting offboard mode")
    await drone.offboard.start()

    # Start the orbiting function immediately after stabilization
    await orbit(drone, orbit_radius=10, orbit_speed=2)

    print("-- Landing")
    await drone.actio
