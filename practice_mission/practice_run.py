import asyncio
from mavsdk import System
from mavsdk.offboard import VelocityBodyYawspeed

async def detect_person():
    """
    Mock function to detect a person. This should be replaced with actual 
    sensor or vision detection logic.
    """
    # Simulating person detection with a 10% probability every second
    await asyncio.sleep(1)
    return False  # Change to True to simulate person detection

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
        # Use async generator for GPS info
        async for gps in drone.telemetry.gps_info():
            print(f"GPS: {gps}")

            if gps.num_satellites < 5:  # Ensure a good number of satellites
                print("Insufficient GPS satellites.")
                return

            break  # Break after getting the first GPS info

        # Use async generator for battery info
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

        # Define a velocity setpoint (x, y, z, yaw) in meters per second
        velocity_body_yawspeed = VelocityBodyYawspeed(0, 0, 0, 0)  # Hover in place
        await drone.offboard.set_velocity_body(velocity_body_yawspeed)

        # Start loitering (orbiting) for up to 30 seconds or until person is detected
        await loiter_and_detect(drone)

        print("-- Landing")
        await drone.action.land()

    except Exception as e:
        print(f"Error: {e}")


async def loiter_and_detect(drone):
    """
    Loiter the drone in a circle and detect for a person. If a person is detected,
    perform a rapid up-down movement. If no detection, land after 30 seconds.
    """
    print("Starting loitering...")

    orbit_task = asyncio.create_task(orbit(drone, orbit_height=10, yaw_behavior="HOLD_YAW"))

    start_time = asyncio.get_event_loop().time()
    person_detected = False

    while asyncio.get_event_loop().time() - start_time < 30:
        person_detected = await detect_person()

        if person_detected:
            print("Person detected! Performing rapid altitude change.")
            await perform_rapid_altitude_change(drone)
            break

        await asyncio.sleep(1)  # Check for person detection every second

    # If no person is detected, cancel orbit after 30 seconds
    if not person_detected:
        orbit_task.cancel()

async def perform_rapid_altitude_change(drone):
    """
    Performs rapid altitude changes for 10 seconds if a person is detected.
    """
    print("-- Performing rapid up-down movement")
    for _ in range(5):  # Move up and down 5 times
        await drone.action.goto_location(latitude_deg=float("nan"),
                                         longitude_deg=float("nan"),
                                         absolute_altitude_m=12)  # Move up
        await asyncio.sleep(1)
        await drone.action.goto_location(latitude_deg=float("nan"),
                                         longitude_deg=float("nan"),
                                         absolute_altitude_m=10)  # Move down
        await asyncio.sleep(1)

    print("-- Returning to loiter mode")
    await orbit(drone, orbit_height=10, yaw_behavior="HOLD_YAW")

'''
Loitering Function
'''
async def orbit(drone, orbit_height, yaw_behavior):
    print('Orbiting at 10m height from the ground')
    await drone.action.do_orbit(
        radius_m=10,
        velocity_ms=2,
        yaw_behavior=yaw_behavior,
        latitude_deg=float("nan"),
        longitude_deg=float("nan"),
        absolute_altitude_m=orbit_height
    )
    # Simulating loitering for 30 seconds or until interrupted by detection
    await asyncio.sleep(30)

if __name__ == "__main__":
    loop = asyncio.get_event_loop()
    loop.run_until_complete(main())
