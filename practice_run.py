#!/user/bin/env python3
from mavsdk import System
'''
Create Take Off function
'''

'''
Create Landing Function
'''

#loitering function
async def loiter_in_rotation(drone, position):
    

# main running function
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
