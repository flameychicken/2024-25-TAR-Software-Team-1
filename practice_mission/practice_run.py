import asyncio
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

        print("-- Taking off")
        await self.drone.action.takeoff()

        await asyncio.sleep(5)  # Wait for the drone to gain some altitude

        print("-- Setting initial setpoint to move forward")
        current_position = await self.get_current_position()  # Get current position

        # Move forward for 10 seconds
        print("-- Flying forward for 10 seconds")
        await self.fly_forward(current_position, duration=10)

        print("-- Landing")
        await self.drone.action.land()

    async def get_current_position(self):
        async for position in self.drone.telemetry.position():
            return position  # Return the latest position from the async generator

    async def fly_forward(self, current_position, duration: float):
        start_time = asyncio.get_event_loop().time()
        forward_distance = 5.0  # Forward distance in meters
        end_time = start_time + duration

        while asyncio.get_event_loop().time() < end_time:
            elapsed_time = asyncio.get_event_loop().time() - start_time
            north_offset = (forward_distance / duration) * elapsed_time

            # Set new position using the current position and north offset
            await self.drone.offboard.set_position_ned(PositionNedYaw(
                current_position.north_m + north_offset,  # Move north
                current_position.east_m,                   # Keep the same east position
                current_position.down_m,                   # Maintain altitude
                0.0                                       # Yaw angle
            ))

            await asyncio.sleep(0.1)  # Sleep to allow time for processing

if __name__ == "__main__":
    controller = DroneController()
    loop = asyncio.get_event_loop()
    loop.run_until_complete(controller.demo1())
