
import asyncio
import commands
from commands import DroneType
import sys
from mavsdk.offboard import PositionNedYaw
from argumentParser import argumentParser
import time
import globals
import rospy
from std_msgs.msg import String
import threading
import signal
import sys
from functools import partial
import utils
stop_thread = False
takeOff = False
def callback(data):
    sim_time = data.clock
    globals.simulationTime = sim_time.secs
    #print("simulation time : {}, start time : {}, mission duration: {}".format(globals.simulationTime, globals.missionStartTime, globals.missionDuration))
    if stop_thread:
        rospy.signal_shutdown("Mission cocmpleted")
    #print("Simulation time: ", sim_time.secs, " seconds")

def subscriber_thread():
    rospy.Subscriber("/clock", String, callback)
    rospy.spin()



async def main():

    # Start the subscriber thread
    isTakeoff = False
    drone, sub_thread, position_future = None, None, None
    droneType, missionDuration, takeoffAltitude, missionAmplitude, missionDirection, vehicleCount, takeoffDuration = argumentParser()
    globals.missionDuration = missionDuration
    globals.missionAmplitude = missionAmplitude
    drone = await commands.connect(droneType, vehicleCount)
    if droneType == DroneType.OBSERVER:
        utils.updateLastMissionNumber()
    globals.drone = drone

    def takeoffHandler(*args):
        print("Takeoff command")
        globals.isTakeoffStarted = True

    def missionStartHandler(*args):
        print("Mission start command")
        globals.isMissionStarted = True

    def observationStopHandler(*args):
        print("Observation stop command")
        globals.isMissionStarted = False

    def observationExitHandler(*args):
        globals.isExit = True
        globals.isStartLand = True

    def returnToLaunchHandler(*args):
        globals.returnToLaunch = True

    def startLandHandler(*args):
        print("Landing started")
        globals.isStartLand = True
        
    signal.signal(signal.SIGUSR1+5, partial(takeoffHandler , [isTakeoff]))
    signal.signal(signal.SIGUSR1+6, partial(missionStartHandler , [isTakeoff]))
    signal.signal(signal.SIGUSR1+8, partial(observationExitHandler , [isTakeoff]))
    signal.signal(signal.SIGUSR1+4, partial(startLandHandler , [isTakeoff]))
    while(1):
        if globals.isTakeoffStarted: 
            break
        pass
    def signal_handler(*args):
        if sub_thread:
            sub_thread.join()
        if position_future:
            position_future.cancel()
        sys.exit(0)

    signal.signal(signal.SIGINT, partial(signal_handler, [drone, sub_thread, position_future]))
    await commands.arm(drone)
    await commands.takeoff(drone, takeoffAltitude)
    await commands.ensureTakeoffIsCompleted(drone, takeoffAltitude)
    #future = asyncio.ensure_future(commands.print_odometry3(drone))

    globals.isMultipleVehicle = False if vehicleCount == 1 else True
    if droneType == DroneType.TARGET:
        signal.signal(signal.SIGUSR1+7, partial(returnToLaunchHandler , [isTakeoff]))          
        rospy.init_node("get_simulation_time")
        #future = asyncio.ensure_future(commands.print_position(drone))

        await commands.setOffBoardMode(drone, takeoffAltitude)
        if globals.isMultipleVehicle:
            print("Waiting for the trajectory start command")
            while globals.isMissionStarted == False:
                pass
            globals.isSaveToFile = True
            await commands.print_odometry(drone, droneType, 0)
            time.sleep(0.1)
            sub_thread = threading.Thread(target=subscriber_thread)
            sub_thread.start()
            globals.isMissionStarted = False
        await commands.makeTrajectory(drone, missionAmplitude, missionDirection, missionDuration, takeoffAltitude, 0)
        #future.cancel()
    else:
        signal.signal(signal.SIGUSR1+7, partial(observationStopHandler , [isTakeoff]))
        await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, -takeoffAltitude, 2.0))
        await drone.offboard.start()
        rospy.init_node(name = "image_viewer")
        if globals.isMultipleVehicle:
            while globals.isMissionStarted == False:
                pass

            globals.isSaveToFile = True
            await commands.print_odometry(drone, droneType, 0)
            time.sleep(0.1)
            sub_thread = threading.Thread(target=subscriber_thread)
            sub_thread.start()
        commands.observeDrone(missionDuration)

    print("Trajectory is completed")
    totalDistance = 0
    count = 1
    if droneType == DroneType.TARGET:
        while totalDistance < 400:
            print("Moving to the next destination")
            totalDistance = totalDistance + 5
            await drone.offboard.set_position_ned(PositionNedYaw(totalDistance, 0.0, -takeoffAltitude, 2.0))
            while globals.isMissionStarted == False:
                if globals.returnToLaunch:
                    await drone.offboard.set_position_ned(PositionNedYaw(0, 0.0, -takeoffAltitude, 2.0))
                    totalDistance = 500
                    break
                pass
            if totalDistance == 500:
                break
            globals.isMissionStarted = False
            globals.isSaveToFile = True
            await commands.print_odometry(drone, droneType, count)
            time.sleep(0.1)
            globals.isTrajectoryStarted = False
            globals.missionStartTime = 0
            globals.tourStarted = False
            await commands.makeTrajectory(drone, missionAmplitude, missionDirection, missionDuration, takeoffAltitude, totalDistance)
            print("Trajectory is completed")
            count = count + 1

    while globals.isStartLand == False:
        pass
    await commands.stopOffBoardMode(drone)
    await commands.land(drone)
    print("Mission completed")
    if(droneType == DroneType.TARGET):
        print("Positions is printing!")
        #analysis.showGraph(False if vehicleCount == 1 else True)
    print("stop")
    global stop_thread
    stop_thread = True
    exit()
    #await asyncio.sleep(40)

if __name__ == "__main__":

    loop = asyncio.get_event_loop()
    loop.run_until_complete(main())
    loop.close()

# TO DO VİDEO SAYILARI