
import asyncio
import commands
from commands import DroneType
import sys
from mavsdk.offboard import PositionNedYaw
from argumentParser import argumentParser
import analysis
import time
import globals

async def main():
    droneType, missionDuration, takeoffAltitude, missionAmplitude, missionDirection, vehicleCount, takeoffDuration = argumentParser()
    drone = await commands.connect(droneType, vehicleCount)
    await commands.arm(drone)
    await commands.takeoff(drone, takeoffAltitude)
    globals.isMultipleVehicle = False if vehicleCount == 1 else True
    time.sleep(takeoffDuration)
    if droneType == DroneType.TARGET:
        future = asyncio.ensure_future(commands.print_position(drone))
        await commands.makeTrajectory(drone, missionAmplitude, missionDirection, missionDuration, takeoffAltitude)
        future.cancel()
    else:
        await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, -takeoffAltitude, 10.0))
        await drone.offboard.start()
        commands.observeDrone(missionDuration)
    await commands.land(drone)
    print("Mission completed")
    if(droneType == DroneType.TARGET):
        print("Positions is printing!")
        analysis.showGraph(False if vehicleCount == 1 else True)
    #await asyncio.sleep(40)

if __name__ == "__main__":
    loop = asyncio.get_event_loop()
    loop.run_until_complete(main())
    loop.close()
