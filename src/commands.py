from mavsdk import System
from mavsdk.offboard import (PositionNedYaw, VelocityNedYaw, OffboardError)
import asyncio
from enum import Enum
from trajectoryGenerator import eightTrajectoryGenerator, Position, TrajectoryDirection
import time
import rospy
from cameraOperations import ROSImageViewer
from mavsdk.offboard import (OffboardError, VelocityNedYaw)
import globals
from gazebo_msgs.srv import GetPhysicsProperties
import utils
class DroneType(Enum):
    
    TARGET = 1
    OBSERVER = 2    

DroneType = Enum('DroneType', ['TARGET', 'OBSERVER'])

async def go_to_position(drone, latitude, longitude, altitude):
    print("Going to position...")
    await drone.action.goto_location(latitude, longitude, altitude, 0)
    await asyncio.sleep(1)

async def connect(droneType, vehicleCount):

    if vehicleCount == 1:
        print("Connecting to Target Drone")
        mavlinkAdress = "udp://:14540"
        port = 50040
    else:
        if droneType == DroneType.TARGET:
            print("Connecting to Target Drone")
            mavlinkAdress = "udp://:14541"
            port = 50041
        else:
            print("Connecting to Observer Drone")
            mavlinkAdress = "udp://:14540"
            port = 50040
    drone = System(mavsdk_server_address="0.0.0.0", port=port)
    await drone.connect(mavlinkAdress)
    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("---Connected to drone!---")
            break
    return drone

async def arm(drone):
    print("Waiting for drone to have a global position estimate...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("---Global position state is good enough for flying.---")
            break
    print("Arming...")
    while 1:
        try:
            await drone.action.arm()
        except:
            pass
        else:
            break

async def takeoff(drone, altitude):
    print("Taking off...")
    await drone.action.set_takeoff_altitude(altitude)
    await drone.action.takeoff()

async def land(drone):
    print("Landing...")
    await drone.action.land()
    await asyncio.sleep(1)
    print("---Landing is started!---")

async def getGroundSpeed(drone):
    async for speed in drone.telemetry.ground_speed_ned():
        return speed

async def setOffBoardMode(drone, takeoffAltitude):
    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, -takeoffAltitude, 2.0))
    print("-- Starting offboard")
    try:
        await drone.offboard.start()
    except OffboardError as error:
        print(f"Starting offboard mode failed with error code: {error._result.result}")
        print("-- Disarming")
        await drone.action.disarm()
        return

async def stopOffBoardMode(drone):
    print("-- Stopping offboard")
    try:
        await drone.offboard.stop()
    except OffboardError as error:
        print(f"Stopping offboard mode failed with error code: {error._result.result}")

async def makeTrajectory(drone, amplitude, direction, missionDuration, takeoffAltitude, positionX):
    dronePosition = Position(0, 0, 0, 0)
    frequency = 0.05
    offSetX = positionX
    offSetY = 0.0
    globals.positionEast = offSetY
    globals.positionNorth = offSetX
    globals.positionDown = -takeoffAltitude
    yaw = 5.0
    isFirstCall = True
    if globals.isMultipleVehicle:
        globals.missionStartTime = globals.simulationTime
    else:
        globals.missionStartTime = time.time()
    elapsed_time = 0
    positionYaw = 2.0
    print("Mission started")
    #print("mission start time: {}".format(globals.missionStartTime))
    count = 0
    globals.isTrajectoryStarted = True
    while elapsed_time < missionDuration:
        if(globals.missionStartTime == 0):
            globals.missionStartTime = globals.simulationTime
        if count == 1:
            break
        #print("Elapsed time: {}".format(elapsed_time))
        if isFirstCall:
            isFirstCall = False
            eightTrajectoryGenerator(amplitude, frequency, elapsed_time, dronePosition)
            if(direction == TrajectoryDirection.X):
                globals.positionNorth = offSetX + dronePosition.x
                globals.positionEast = offSetY + dronePosition.y
                globals.positionDown = -takeoffAltitude
            else:
                globals.positionNorth = offSetX
                globals.positionEast = offSetY + dronePosition.y
                globals.positionDown = -takeoffAltitude - dronePosition.x
        else:
            #print(elapsed_time)
            if eightTrajectoryGenerator(amplitude, frequency, elapsed_time, dronePosition) == True:
                count = count +1
            if(direction == TrajectoryDirection.X):
                globals.positionNorth = offSetX + dronePosition.x
                globals.positionEast = offSetY + dronePosition.y
                globals.positionDown = -takeoffAltitude
            else:
                globals.positionNorth = offSetX
                globals.positionEast = offSetY + dronePosition.y
                globals.positionDown = -takeoffAltitude - dronePosition.x
        if globals.isMultipleVehicle:
            elapsed_time = globals.simulationTime - globals.missionStartTime  # in seconds
        else:
            elapsed_time = time.time() - globals.missionStartTime
        await drone.offboard.set_position_ned(PositionNedYaw(globals.positionNorth, globals.positionEast, globals.positionDown, positionYaw))
    #await stopOffBoardMode(drone)
    globals.isTrajectoryStarted = False

def observeDrone(missionDuration):
    viewer = ROSImageViewer(missionDuration)
    rospy.spin()
    print("observation completed")
    return 

async def print_position(drone):
    if globals.isMultipleVehicle:
        with open("positions", "w+") as f:
            async for position in drone.telemetry.position():
                #print(position)
                f.write(position.__str__() + "\n")
    else:
        with open("positions", "w+") as f:
            with open("nedPositions", "w+") as f2:
                f.truncate()
                f2.truncate()
                async for position in drone.telemetry.position():
                    if globals.isTrajectoryStarted:
                        #print(position)
                        string = str(globals.positionNorth) + "," + str(globals.positionEast) + "," + str(globals.positionDown) + "\n"
                        f.write(position.__str__() + "\n")
                        f2.write(string)

async def ensureTakeoffIsCompleted(drone, takeoffAltitude):
    async for position in drone.telemetry.position():
        if position.relative_altitude_m >= takeoffAltitude -1:
            break
        
async def print_odometry(drone, droneType, count):
    await asyncio.sleep(1)
    missionNumber = str(utils.getMissionNumber())
    if droneType == DroneType.TARGET:
        with open("../observations/" + missionNumber + "/target" + str(count), "w+") as f:
            f.truncate()
            async for odometry in drone.telemetry.odometry():
                if globals.isSaveToFile:
                    try:
                        f.write(odometry.__str__())
                        f.write("\n")
                    except Exception as e:
                        print("An exception occurred : {}".format(e)) 
                    globals.isSaveToFile = False
                    break

    else:
        with open("../observations/"+ missionNumber + "/observer", "w+") as f2:
            f2.truncate()        
            async for odometry in drone.telemetry.odometry():
                if globals.isSaveToFile:
                    try:
                        f2.write(odometry.__str__())
                        f2.write("\n")
                    except Exception as e:
                        print("An exception occurred : {}".format(e)) 
                    globals.isSaveToFile = False
                    break
                return
async def print_flight_mode(drone):
    """ Prints the flight mode when it changes """

    previous_flight_mode = None

    async for flight_mode in drone.telemetry.flight_mode():
        if flight_mode != previous_flight_mode:
            previous_flight_mode = flight_mode
            print(f"Flight mode: {flight_mode}")

async def moveToInitialPoint(drone):
    await drone.offboard.set_position_ned(PositionNedYaw(0, 0.0, 0.0, 2.0))
