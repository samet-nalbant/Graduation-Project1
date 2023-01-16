import argparse
from commands import DroneType
from trajectoryGenerator import TrajectoryDirection

class DroneTypeAction(argparse.Action):
    def __call__(self, parser, namespace, values, option_string=None):
        if values.lower() != "target" and values.lower() != "observer":
            raise ValueError("Invalid drone type!")
        setattr(namespace, self.dest, values.upper())

class MissionDurationAction(argparse.Action):
    def __call__(self, parser, namespace, values, option_string=None):
        if values <=0:
            raise ValueError("Mission duration must be bigger than 0!")
        setattr(namespace, self.dest, values)

class MissionAltitudeAction(argparse.Action):
    def __call__(self, parser, namespace, values, option_string=None):
        if values <=0:
            raise ValueError("Mission altitude must be bigger than 0!")
        setattr(namespace, self.dest, values)

class MissionAmplitudeAction(argparse.Action):
    def __call__(self, parser, namespace, values, option_string=None):
        if values <=0:
            raise ValueError("Mission amplitude must be bigger than 0!")
        setattr(namespace, self.dest, values)

class MissionDirectionAction(argparse.Action):
    def __call__(self, parser, namespace, values, option_string=None):
        if values.upper() != "X" and values.upper() != "Y":
            raise ValueError("Mission direction must be X or Y!")
        setattr(namespace, self.dest, TrajectoryDirection.X if values.upper() == "X" else TrajectoryDirection.Y)       

class VehicleCountAction(argparse.Action):
    def __call__(self, parser, namespace, values, option_string=None):
        if values <= 0 or values > 2:
            raise ValueError("Vehicle number must be 1 or 2!")
        setattr(namespace, self.dest, values)  

class TakeoffDurationAction(argparse.Action):
    def __call__(self, parser, namespace, values, option_string=None):
        if values <= 5:
            raise ValueError("Take off duration must be greater than 5!")
        setattr(namespace, self.dest, values)  

def argumentParser():
    parser = argparse.ArgumentParser()
    parser.add_argument("--droneType", action=DroneTypeAction, required=True, type=str)
    parser.add_argument('--missionDuration', action=MissionDurationAction, type=int, required=False, default=60)
    parser.add_argument("--missionAltitude", action=MissionAltitudeAction, type=int, required=False, default=10)
    parser.add_argument("--missionAmplitude",action=MissionAmplitudeAction, type=int, required=False, default=5)
    parser.add_argument("--missionDirection",action=MissionDirectionAction, type=str, required=False, default=TrajectoryDirection.Y)
    parser.add_argument("--vehicleCount",action=VehicleCountAction, type=int, required=False, default=2)
    parser.add_argument("--takeoffDuration",action=TakeoffDurationAction, type=int, required=False, default=10)
    args = parser.parse_args()
    print("Drone Type: {}".format(args.droneType))
    print("Mission duration: {}".format(args.missionDuration))
    print("Mission Altitude: {}".format(args.missionAltitude))
    print("Mission Amplitude: {}".format(args.missionAmplitude))
    print("Mission Direction: {}".format(args.missionDirection))
    print("Vehicle Count: {}".format(args.vehicleCount))
    print("Takeoff Duration: {}".format(args.takeoffDuration))
    return DroneType.TARGET if args.droneType == "TARGET" else DroneType.OBSERVER, args.missionDuration, args.missionAltitude, args.missionAmplitude, args.missionDirection, args.vehicleCount, args.takeoffDuration