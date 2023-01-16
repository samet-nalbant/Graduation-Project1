
import asyncio
import commands
from commands import DroneType
import sys
from mavsdk.offboard import PositionNedYaw
from argumentParser import argumentParser
import analysis
import time
import globals
import rospy
from std_msgs.msg import String
import threading
import signal
import sys
from functools import partial
def callback(data):
    sim_time = data.clock
    globals.simulationTime = sim_time.secs
    #print("simulation time : {}, start time : {}, mission duration: {}".format(globals.simulationTime, globals.missionStartTime, globals.missionDuration))
    if globals.simulationTime - globals.missionStartTime > globals.missionDuration and globals.missionStartTime != 0:
        rospy.signal_shutdown("Mission cocmpleted")
    #print("Simulation time: ", sim_time.secs, " seconds")

def subscriber_thread():
    rospy.Subscriber("/clock", String, callback)
    rospy.spin()

async def main():

    # Start the subscriber thread

    drone, sub_thread, position_future = None, None, None
    droneType, missionDuration, takeoffAltitude, missionAmplitude, missionDirection, vehicleCount, takeoffDuration = argumentParser()
    globals.missionDuration = missionDuration
    globals.missionAmplitude = missionAmplitude
    drone = await commands.connect(droneType, vehicleCount)
    def signal_handler( *args):
        if sub_thread:
            sub_thread.stop()
        if position_future:
            position_future.cancel()
        sys.exit(0)
    signal.signal(signal.SIGINT, partial(signal_handler, [drone, sub_thread, position_future]))
    await commands.arm(drone)
    await commands.takeoff(drone, takeoffAltitude)
    await commands.ensureTakeoffIsCompleted(drone, takeoffAltitude)
    globals.isMultipleVehicle = False if vehicleCount == 1 else True
    if droneType == DroneType.TARGET:
        rospy.init_node("get_simulation_time")
        future = asyncio.ensure_future(commands.print_position(drone))
        if globals.isMultipleVehicle:
            sub_thread = threading.Thread(target=subscriber_thread)
            sub_thread.start()
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
    print("stop")
    sub_thread.stop()
    #await asyncio.sleep(40)

if __name__ == "__main__":

    loop = asyncio.get_event_loop()
    loop.run_until_complete(main())
    loop.close()
