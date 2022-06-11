#!/usr/bin/env python2

import rospy, cv2

from sensor_msgs.msg import Image
from std_msgs.msg import String, Bool
import numpy as np
#from tensorflow.keras.models import load_model
#import tensorflow as tf
import os
#from sign_rec.srv import check, data


# Constant definition
frameWidth = 640  # CAMERA RESOLUTION
frameHeight = 480
brightness = 180

#device = tf.config.list_physical_devices('GPU')
#tf.config.experimental.set_memory_growth(device[0], True)
#tf.config.experimental.set_virtual_device_configuration(device[0], [tf.config.experimental.VirtualDeviceConfiguration(memory_limit=1024)])

#model = load_model("/home/puzzlebot/puzzlebot_ws/src/puzzlebot_vision/src/puzzlebot_model.h5")


ROS_LABEL_PUBLISHER = "/puzzlebot/traffic_signals/predictions/"


ROS_IMAGE_OUTPUT_TOPIC = '/puzzlebot_vision/traffic_signals/image_segmentation'
ROS_SIGNAL_DETECT_TOPIC = '/puzzlebot_vision/traffic_signals/signal_found'




IMG_SHAPE = (150,150,1)
IMG_TUPPLE_SHAPE = (150,150)
BATCH_SIZE= 128
dims = 150

RATE = 30

class DetectStop():


    def __init__(self):

        self.image = None
        self.flag = None

        self.imgSub = rospy.Subscriber(ROS_IMAGE_OUTPUT_TOPIC,Image,self.imageCallback)
        self.flagSub = rospy.Subscriber(ROS_SIGNAL_DETECT_TOPIC,Bool,self.flagCallback)

        self.labelPub = rospy.Publisher(ROS_LABEL_PUBLISHER,String,queue_size=1)

        rospy.init_node("puzzlebot_predictor_node")
        self.rate = rospy.Rate(RATE)



    def imageCallback(self,img):
        self.image = np.frombuffer(img.data, dtype=np.uint8).reshape(img.height, img.width, -1)
        rospy.loginfo("sutff")


    def flagCallback(self,msg):
        self.flag = msg.data
        




    def imageProcessing(self,image):
        image = image.astype(np.uint8)
        gray = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
        gray = cv2.equalizeHist(gray)
        gray = cv2.resize(gray,IMG_TUPPLE_SHAPE)
        gray = gray.reshape(1,dims,dims,1)
        return gray.astype(np.float64)/255



    def getCalssName(classNo):
        labels = {0: 'stop_signal', 1: 'aplastame', 2: 'right_signal', 3: 'left_signal', 4: 'up_signal', 5: 'around_signal'}
        return labels[classNo]




    # function callbacks 

    def run(self):

        while not rospy.is_shutdown():
            if self.image is None:
                self.rate.sleep()
                continue

            
            
            frame = self.image
            img = self.imageProcessing(frame)


            #prediction = model.predict(img)
            #index = np.argmax(prediction)
            #label = self.getCalssName(index)

            rospy.loginfo(img.shape)
           
            self.image = None

            


if __name__ == '__main__':
    predictor = DetectStop()
    predictor.run()