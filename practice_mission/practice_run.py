import asyncio
from mavsdk import System
from mavsdk.offboard import PositionNedYaw, OffboardError


class DroneController:
    def __init__(self):
        self.drone = System()
          
    async def practice_run(self, port: str = 'udp://:14540'):
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

        # Setting initial position at 1 meter altitude
        await self.drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, -1.0, 0.0))
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

        # Fly the square pattern twice
        await self.fly_square(2)  # `n=2` for now

        # After completing the square, land the drone
        print("-- Landing")
        await self.drone.action.land()

    async def fly_square(self, n: int):
        # Define the square path by setting the NED frame positions
        square_path = [
            PositionNedYaw(5.0, 0.0, -1.0, 0.0),    # Move 5 meters North
            PositionNedYaw(5.0, 5.0, -1.0, 90.0),   # Move 5 meters East
            PositionNedYaw(0.0, 5.0, -1.0, 180.0),  # Move 5 meters South
            PositionNedYaw(0.0, 0.0, -1.0, 270.0)   # Move 5 meters West back to start
        ]

        # Repeat the square pattern `n` times
        for i in range(n):
            print(f"-- Starting square loop {i + 1}")
            for j, position in enumerate(square_path):
                print(f"-- Flying to waypoint {j + 1} of loop {i + 1}")
                await self.drone.offboard.set_position_ned(position)
                await asyncio.sleep(5)  # Wait to reach each waypoint


if __name__ == "__main__":
    controller = DroneController()
    loop = asyncio.get_event_loop()
    loop.run_until_complete(controller.practice_run())
