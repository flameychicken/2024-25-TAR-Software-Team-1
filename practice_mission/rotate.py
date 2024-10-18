import asyncio
import math
from mavsdk import System
from mavsdk.offboard import PositionNedYaw, OffboardError

class DroneController:
    def __init__(self):
        self.drone = System()

    async def demo1(self, port: str = 'udp://:14540'):
        await self.drone.connect(system_address=port)

        print("Waiting for connection...")
        async for state in self.drone.core.connection_state():
            if state.is_connected:
                print("-- connection successful")
                break

        print("Waiting for drone to have a global position estimate...")
        async for health in self.drone.telemetry.health():
            if health.is_global_position_ok and health.is_home_position_ok:
                print("-- Global position estimate OK")
                break

        await asyncio.sleep(1)  # Allow time for the drone to stabilize

        print("-- Arming")
        await self.drone.action.arm()

        await asyncio.sleep(0.5)

        print("-- Setting initial setpoint")
        
        # Awaiting the first value from the position async generator
        async for position in self.drone.telemetry.position():
            current_position = position
            break

        # Set the initial position above ground
        await self.drone.offboard.set_position_ned(PositionNedYaw(
            0.0,  # North in NED frame
            0.0,  # East in NED frame
            -1.0,  # Down in NED frame
            0.0    # Yaw angle
        ))

        print("-- Initial setpoint set successfully")

        print("-- Starting offboard")
        try:
            await self.drone.offboard.start()
            print("-- Offboard mode started successfully")
        except OffboardError as error:
            print(f"Starting offboard mode failed with error code: {error._result.result}")
            print("-- Disarming")
            await self.drone.action.disarm()
            return

        # Fly in a circle for 30 seconds
        await self.fly_in_circle(radius=5, duration=30)

        print("-- Landing")
        await self.drone.action.land()

    async def fly_in_circle(self, radius: float, duration: float):
        """
        Fly in a circular path for the specified duration.
        :param radius: Radius of the circular path in meters.
        :param duration: Duration to fly in the circle in seconds.
        """
        start_time = asyncio.get_event_loop().time()
        while asyncio.get_event_loop().time() - start_time < duration:
            # Get current position
            async for position in self.drone.telemetry.position():
                current_position = position
                break

            # Calculate the new setpoint for circular motion
            elapsed_time = asyncio.get_event_loop().time() - start_time
            angle = (elapsed_time / duration) * 2 * math.pi  # Calculate angle in radians
            north_offset = radius * math.cos(angle)  # NED north component
            east_offset = radius * math.sin(angle)   # NED east component

            # Set the new position relative to current position
            await self.drone.offboard.set_position_ned(PositionNedYaw(
                current_position.north_m + north_offset,  # Update north offset
                current_position.east_m + east_offset,    # Update east offset
                -1.0,                                      # Maintain altitude
                0.0                                        # Maintain yaw
            ))

            await asyncio.sleep(0.1)  # Update position every 100 ms

if __name__ == "__main__":
    controller = DroneController()
    loop = asyncio.get_event_loop()
    loop.run_until_complete(controller.demo1())
