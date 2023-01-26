import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import os
import re
import globals
from datetime import datetime
import signal
import os
import time
import utils

class ROSImageViewer:
    isFirstCall = True
    start_time,elapsed_time, observationDuration, lastMissionNumber, trajectoryCount = 0, 0, 0, 0, 0
    isRunning = False
    writer = None
    count = 1
    def __init__(self, observationDuration):
        self.start_time = globals.simulationTime
        self.observationDuration = observationDuration
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/iris_opt_flow_0/c920/image_raw', Image, self.image_callback)
        self.isFirstCall = True
        self.isRunning = True
        self.lastMissionNumber = str(utils.getMissionNumber())

        
    def image_callback(self, msg):
        if self.isRunning:
            if self.start_time == 0:
                self.start_time = globals.simulationTime
            self.elapsed_time = globals.simulationTime - self.start_time  # in seconds
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            #print(self.isFirstCall)
            if self.isFirstCall:
                self.isFirstCall = False
                dt = datetime.now()
                ts = datetime.timestamp(dt)
                videoPath = "../observations/" +self.lastMissionNumber + "/" + str(self.trajectoryCount)+".mp4"
                self.writer = cv2.VideoWriter(videoPath, cv2.VideoWriter_fourcc('m', 'p', '4', 'v'), 20, (image.shape[1], image.shape[0]))
            else:
                self.writer.write(image)
                resized = cv2.resize(image, (800,600), interpolation = cv2.INTER_AREA)
                cv2.imshow('Observer Camera View', resized)
                key = cv2.waitKey(1)
                if globals.isMissionStarted == False:
                    #self.image_sub.unregister()
                    cv2.destroyAllWindows()
                    self.isRunning = False
                    self.count = self.count + 1
                    self.trajectoryCount = self.trajectoryCount +1
                    #rospy.signal_shutdown("Observation completed")  # shut down the ROS node
        else:
            while globals.isMissionStarted == False:
                if globals.isExit:
                    self.writer.release()
                    self.image_sub.unregister()
                    rospy.signal_shutdown("Observation completed")  # shut down the ROS node
                    
                    pid = os.getpid()
                    os.kill(pid, signal.SIGUSR1) 
                pass
            self.isRunning = True
            self.isFirstCall = True
            self.start_time,self.elapsed_time, self.observationDuration = 0, 0, 0
            globals.isSaveToFile = True
            globals.count = globals.count + 1

            