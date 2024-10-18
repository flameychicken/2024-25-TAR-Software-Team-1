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

        print("-- Setting initial setpoint")
        
        # Awaiting the first value from the position async generator
        async for position in self.drone.telemetry.position():
            # Getting the current position
            current_position = position
            break

        await self.drone.offboard.set_position_ned(PositionNedYaw(
            current_position.north_m + 1.0,  # Offset slightly from current position
            current_position.east_m,
            current_position.down_m,
            0.0
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

        # Add your additional flight logic here (e.g., moving, hovering, etc.)
        
        # Example of hovering for a while before landing
        await asyncio.sleep(10)  # Hover for 10 seconds

        print("-- Landing")
        await self.drone.action.land()

if __name__ == "__main__":
    controller = DroneController()
    loop = asyncio.get_event_loop()
    loop.run_until_complete(controller.demo1())
