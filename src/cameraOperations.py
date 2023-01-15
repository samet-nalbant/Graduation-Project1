import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import os
import re
import time

def findLastVideoNumber():
    videoDirectoryPath = "../observations"
    if os.path.exists("../observations") == False:
        os.mkdir("../observations")

    numberList = []
    for path in os.listdir(videoDirectoryPath):
        if os.path.isfile(os.path.join(videoDirectoryPath, path)):
            if path.endswith(".mp4") and path.startswith("video"):
                number = re.findall('[0-9]+', path)
                numberList.append(number[0])

    if len(numberList) != 0:    
        numberList.sort(reverse=True)
        newVideoNumber = int(numberList[0])+1
    else:
        newVideoNumber = 0
    return newVideoNumber

class ROSImageViewer:
    isFirstCall = True
    start_time,elapsed_time, observationDuration = 0, 0, 0
    def __init__(self, observationDuration):
        print("initialization")
        self.start_time = time.time()
        self.observationDuration = observationDuration
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/iris_opt_flow_0/c920/image_raw', Image, self.image_callback)
        self.isFirstCall = True
        
    def image_callback(self, msg):
        global writer
        self.elapsed_time = time.time() - self.start_time  # in seconds
        print(self.elapsed_time)
        if self.elapsed_time > self.observationDuration:
            time.sleep(1)  # add a delay of 1 second
            self.image_sub.unregister()
            cv2.destroyAllWindows()
            rospy.signal_shutdown("Observation completed")  # shut down the ROS node
            return
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        if self.isFirstCall:
            self.isFirstCall = False
            videoPath = "../observations/video" + str(findLastVideoNumber())+".mp4"
            writer = cv2.VideoWriter(videoPath, cv2.VideoWriter_fourcc('m', 'p', '4', 'v'), 20, (image.shape[1], image.shape[0]))
        else:
            writer.write(image)
        resized = cv2.resize(image, (1024,720), interpolation = cv2.INTER_AREA)
        cv2.imshow('image', resized)
        cv2.waitKey(1)    
