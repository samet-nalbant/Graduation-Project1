import os
def getMissionNumber():
    with open('/home/samet/ACTIVE_PLANNING', 'r') as file:
        data = file.read()
        values = data.split("=")
        if values[0] == "LAST_MISSION":
            last_mission_number = int(values[1])
        else:
            last_mission_number = -1
    return last_mission_number

def updateLastMissionNumber():
    last_mission_number = getMissionNumber()
    with open('/home/samet/ACTIVE_PLANNING', 'w') as file:
        file.truncate()
        file.write("LAST_MISSION="+str(last_mission_number+1))

    directory = '../observations/' + str(last_mission_number+1)
    if not os.path.exists(directory):
        os.makedirs(directory)
    return True
