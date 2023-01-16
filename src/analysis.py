import re
import matplotlib.pyplot as plt
import numpy as np
from shapely.geometry import LineString
from sklearn.cluster import KMeans
import math
from geopy.distance import geodesic
import pandas as pd
from scipy.stats import pearsonr
from scipy.spatial.distance import directed_hausdorff
import globals
def findRadius(flight_data):
    # using Ramer-Douglas-Peucker algorithm
    flight_data.remove(flight_data[0])
    flight_data.remove(flight_data[1])
    for position in flight_data:
        if len(position) != 2 or type(position[0]) != float or type(position[1]) != float:
            flight_data.remove(position)
    line = LineString(flight_data)
    simplified_line = line.simplify(0.001)
    simplified_points = np.array(simplified_line.coords)
    kmeans = KMeans(n_clusters=1, random_state=0).fit(simplified_points)
    center = kmeans.cluster_centers_[0]
    distances = np.linalg.norm(simplified_points - center, axis=1)
    radius = np.max(distances)
    print("Radius of 8-shaped graph: ", radius)

def readPositionLogFile(filePath):
    pattern = re.compile(r'Position: \[latitude_deg: (\d+\.\d+), longitude_deg: (\d+\.\d+), absolute_altitude_m: (\d+\.\d+), relative_altitude_m: (\d+\.\d+)\]')
    with open(filePath, 'r') as f:
        contents = f.read()

    matches = pattern.finditer(contents)

    latitudes, longtitudes, altitudes, positions= [], [], [], [[],[]]
    for match in matches:
        if float(match.group(4)) > 1: 
            latitudes.append(float(match.group(1)))
            longtitudes.append(float(match.group(2)))
            altitudes.append(float(match.group(4)))
            temp = []
            if(float(match.group(1)) > 0):
                temp.append(float(match.group(1)))
            if(float(match.group(4)) > 0):
                temp.append(float(match.group(4)))
            if len(temp) == 2:
                positions.append(temp)
    return latitudes, longtitudes, altitudes, positions



def adjustFlightDataToPerfectEightFormation(longtitudes, altitudes):

    min_lon = min(longtitudes)
    min_alt = min(altitudes)
    for i in range(0, len(longtitudes)):
        if longtitudes[i] - min_lon < 1:
            longtitudes[i] = (longtitudes[i] - min_lon)*10**5
        # altitudes[i] = round((altitudes[i] - min_alt), 4)
        # longtitudes[i] = round((longtitudes[i] - min_lon),4)
        else:
            longtitudes[i] = longtitudes[i] - min_lon
        altitudes[i] = altitudes[i] - min_alt
    return longtitudes, altitudes


def readNEDPositions():
    df = pd.read_csv("nedPositions", names=['N', 'E', 'D'], header=None)
    return df

def calculateErrorRate(df, longtitudes, altitudes):
    df['D'] = df['D'] + abs(min(df['D']))
    df['E'] = df['E'] + abs(min(df['E']))
    adjustedLongtitudes, adjustedAltitudes = adjustFlightDataToPerfectEightFormation(longtitudes,altitudes)
    perfect_eight_data = df[['E', 'D']].values
    eight_shape_data = np.column_stack((adjustedLongtitudes, adjustedAltitudes))
    max_alt = max(adjustedAltitudes)
    min_alt = min(adjustedAltitudes)
    min_lon = min(adjustedLongtitudes)
    max_lon = max(adjustedLongtitudes)
    error = np.sqrt(np.mean((perfect_eight_data - eight_shape_data)**2))
    print("Error rate:", error)
    true_value = df[['E', 'D']].values
    predicted_value = np.column_stack((adjustedLongtitudes, adjustedAltitudes))
    #true_value.sort()
    #predicted_value.sort()
    error = np.sqrt(np.mean((true_value - predicted_value)**2))
    error_rate = (error / true_value.mean()) * 100
    print("Error rate: {:.3f}%".format(error_rate))

    corr, _ = pearsonr(perfect_eight_data.flatten(), eight_shape_data.flatten())
    print("Correlation coefficient:", corr)

    print("altitude difference: {}".format((float(max_alt)-float(min_alt))))
    print("amplitude difference: {}".format((float(max_alt)-float(min_alt))/2))
    print("amplitude: {}".format(globals.missionAmplitude))
    # fig, ax = plt.subplots(2,1)
    # ax[0].plot(df['E'], df['D'], label='E-D')
    # ax[1].set_xlim(min_lon, max_lon)
    # ax[1].set_ylim(min_alt, max_alt)
    # ax[1].scatter(adjustedLongtitudes, adjustedAltitudes, marker='o', color='r', zorder=2, s=1)
    # plt.show()
    #findRadius(positions)

def showGraph(isMultipleVehicle):
    latitudes, longtitudes, altitudes, positions = readPositionLogFile("positions")
    if isMultipleVehicle == False:
        df = readNEDPositions()
        fig, ax = plt.subplots(2,1)
        min_lon = min(longtitudes)
        max_lon = max(longtitudes)
        min_alt = min(altitudes)
        max_alt = max(altitudes)
        ax[0].plot(df['E'], df['D'], label='E-D')
        ax[1].set_xlim(min_lon, max_lon)
        ax[1].set_ylim(min_alt, max_alt)
        ax[1].scatter(longtitudes, altitudes, marker='o', color='r', zorder=2, s=1)
        #calculateErrorRate(df, longtitudes, altitudes)

    else:
        fig, ax = plt.subplots()
        min_lon = min(longtitudes)
        max_lon = max(longtitudes)
        min_alt = min(altitudes)
        max_alt = max(altitudes)
        ax.set_xlim(min_lon, max_lon)
        ax.set_ylim(min_alt, max_alt)
        ax.scatter(longtitudes, altitudes, marker='o', color='r', s=1)
    plt.show()
    exit()