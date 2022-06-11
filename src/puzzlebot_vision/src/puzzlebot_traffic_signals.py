#!/usr/bin/env python
# -*- coding: utf-8 -*-
from signal import signal
import cv2
from matplotlib.pyplot import draw
import numpy as np
import rospy, cv_bridge
from sensor_msgs.msg import Image
from std_msgs.msg import Bool



# solamente se usan 2 topicos para detectar la señal y para la imagen
ROS_IMAGE_OUTPUT_TOPIC = '/puzzlebot_vision/traffic_signals/image_segmentation'
ROS_IMAGE_SQUARE_TOPIC = "/puzzlebot_vision/traffic_signals/bounding_boxes"
ROS_SIGNAL_DETECT_TOPIC = '/puzzlebot_vision/traffic_signals/signal_found'


# cargar del servidor de parametros
RATE   =  10
IMG_SCALE_FACTOR = 50


VIDEO_PATH = "/home/luis/Desktop/Puzzlebot_Signals_Video/puzzlebot_record.avi"


class Signal_Idendifier:

    def __init__(self):

        self.bridge = cv_bridge.CvBridge()

        self.imageScaleFactor = IMG_SCALE_FACTOR
        

        # detection flags for potential candidates
        self.signalDetectedPub = rospy.Publisher(ROS_SIGNAL_DETECT_TOPIC, Bool, queue_size = 10)
        self.outputImagePub = rospy.Publisher(ROS_IMAGE_OUTPUT_TOPIC,Image,queue_size=10)
        self.boxesImagePub = rospy.Publisher(ROS_IMAGE_SQUARE_TOPIC,Image,queue_size=10)

        # Initialize a rospy node with the name 'puzzlebot_traffic_lights'.
        rospy.init_node('puzzlebot_traffic_signals')

        pub_rate = RATE
        # Define the ROS node execution rate
        self.rate = rospy.Rate(pub_rate)


    def preprocessImage(self,img):
        # Method used to preprocess the node input image, the image processing must be divided in:
        # 1 - Resize the input image to a specified image scale factor.
        # 2 - Rotate the image if required.
        # 3 - Apply an adequate Gaussian Blur to the image, modify the filter kernel as required.
        # 4 - Return the final processed image

        ##########################################################################################################
        # TODO: Complete the class method definition using the previous description
        ##########################################################################################################

        # Your code here...
        width = int(img.shape[0]*self.imageScaleFactor/100)
        height = int(img.shape[1]*self.imageScaleFactor/100)
        img = cv2.resize(img,(height,width))
        img = cv2.rotate(img,cv2.ROTATE_180)

        ##########################################################################################################
        return img

    def getHoughCircles(self,img):
        gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

        circles_img = cv2.HoughCircles(gray,cv2.HOUGH_GRADIENT,1,100,param1=50,param2=30,minRadius=15,maxRadius=30)
        #circles_img = np.uint16(np.around(circles_img))
        return circles_img





    def pubSegementedImages(self,circles,img):
        images = list()
        rectangles = list()
        outputImg = img.copy()
        if circles is not None:
            circles = np.round(circles[0, :]).astype("int")
            for circle in circles:
                x = circle[0]
                y = circle[1]
                rad = circle[2] + 10
                topLeftCorner = (x-(rad),y-(rad))
                bottomRightCorner =  (x+(rad),y+(rad))
                outputImg = cv2.rectangle(outputImg,topLeftCorner,bottomRightCorner,(255,0,0),5)
                segemented = img.copy()[y-rad:y+rad,x-rad:x+rad,:]
                
                try:
                    segmentedOutput = self.bridge.cv2_to_imgmsg(segemented,encoding="bgr8")
                    self.outputImagePub.publish(segmentedOutput)
                except:
                    continue           
                
            output = self.bridge.cv2_to_imgmsg(outputImg,encoding="bgr8")
            self.boxesImagePub.publish(output)




    def run(self):
        capture = cv2.VideoCapture(VIDEO_PATH)
        while not rospy.is_shutdown():
            
            isTrue, img = capture.read()
            img = self.preprocessImage(img)

            circles = self.getHoughCircles(img)
            flag = True if circles is not None else False
            self.signalDetectedPub.publish(flag)

            self.pubSegementedImages(circles,img)
            self.rate.sleep()


if __name__ == '__main__':
    traffic_lights_detector = Signal_Idendifier()
    try:
        traffic_lights_detector.run()
    except rospy.ROSInterruptException:
        pass