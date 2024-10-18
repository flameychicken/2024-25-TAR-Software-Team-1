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

    # Delay to allow the drone to initialize properly
    await asyncio.sleep(2)  # Wait for 2 seconds

    try:
        # Check GPS info
        async for gps in drone.telemetry.gps_info():
            print(f"GPS: {gps}")

            if gps.num_satellites < 5:  # Ensure a good number of satellites
                print("Insufficient GPS satellites.")
                return

            break  # Break after getting the first GPS info

        # Check battery info
        async for battery in drone.telemetry.battery():
            print(f"Battery: {battery.remaining_percent * 100:.2f}%")

            if battery.remaining_percent < 0.2:  # Check if battery is below 20%
                print("Battery too low to arm.")
                return
            
            break  # Break after getting the first battery info

        print("-- Checking pre-flight conditions...")
        print("-- Arming")
        await drone.action.arm()

        print("-- Taking off")
        await drone.action.takeoff()

        # Wait for the drone to reach a stable altitude
        await asyncio.sleep(5)

        print("-- Setting offboard mode")
        await drone.offboard.start()

        # Fly in a circle
        await fly_in_circle(drone)

        print("-- Landing")
        await drone.action.land()

    except Exception as e:
        print(f"Error: {e}")

async def fly_in_circle(drone):
    """
    Fly in a circular path by sending velocity setpoints.
    """
    print("Starting to fly in a circle...")

    # Define parameters for the circular flight
    radius = 10  # meters
    speed = 2  # meters per second
    duration = 30  # seconds to fly around

    # Calculate the number of iterations based on duration and speed
    iterations = duration // 2  # 2 seconds per iteration

    for _ in range(iterations):
        # Calculate the next setpoint based on circle parameters
        yaw = _ * (360 / iterations)  # Change yaw angle for circular motion
        velocity_body_yawspeed = VelocityBodyYawspeed(speed, 0, 0, yaw)

        await drone.offboard.set_velocity_body(velocity_body_yawspeed)
        await asyncio.sleep(2)  # Send setpoint every 2 seconds

    print("Finished flying in a circle.")

if __name__ == "__main__":
    loop = asyncio.get_event_loop()
    loop.run_until_complete(main())
