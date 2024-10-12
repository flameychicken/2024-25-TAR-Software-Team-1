#!/user/bin/env python3
from mavsdk import System
'''
Create Take Off function
'''

'''
TESTING
Create Landing Function
'''

'''
Create Loitering Function
'''

async def run():

    #Init drone
    drone = System()
    await drone.connect(system_address="udp://:14540")

    print("-- Arming")
    await drone.action.arm()

    print("-- Taking off")
    await drone.action.takeoff()

    await asyncio.sleep(10)

    print("-- Landing")
    await drone.aciton.land()

if __name__ = "__main__":
    asyncio.run(run())
