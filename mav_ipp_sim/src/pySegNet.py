#!/usr/bin/python
# Inkyu Sa, enddl22@gmail.com
# 19/Sep/2017 at Bonn, Flourish Integration week
# GNU GPLv3

import numpy as np
import matplotlib.pyplot as plt
import os.path
import scipy
import scipy.io as sio
import argparse
import math
import cv2
import sys
import time
import rospy
import Queue
import std_srvs.srv
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String,Int32,Int32MultiArray,MultiArrayLayout,MultiArrayDimension,Float32MultiArray


sys.path.append('/usr/local/lib/python2.7/site-packages')
# Make sure that caffe is on the python path:
caffe_root = '/home/masha/catkin_ws/src/weedNet-devel/caffe-segnet-cudnn5/'
sys.path.insert(0, caffe_root + 'python')
import caffe


class segNet(object):
    def __init__(self):
        self.imgSub = rospy.Subscriber("image",Image,self.ImgCallback)
        self.imgPub = rospy.Publisher("image_seg", Image, queue_size=1)
        self.processImgService = rospy.Service("process_image", std_srvs.srv.Empty, self.processImgCallback)
        #self.clsPercPub=rospy.Publisher("clsPerc",Float32MultiArray,queue_size = 1)
        self.bridge = CvBridge()
        self.model='/home/masha/catkin_ws/src/weedNet-devel/SegNet-Tutorial/Models/segnet_ipp_rit18_inference_live.prototxt'
        self.weights='/home/masha/catkin_ws/src/weedNet-devel/SegNet-Tutorial/Models/Inference/rit18_weights.caffemodel'
        self.net = None
        self.inputImg=None
        self.input_shape = None
        self.imgWidth=None
        self.imgHeigh=None
        self.output_shape = None
        self.Soil = [0,0,255]
        self.Plant = [255,0,0]
        self.Weed = [0,255,0]
        self.label_colours = np.array([self.Soil, self.Plant, self.Weed])
        self.imgQ=Queue.Queue(maxsize=1)
        self.outputImg=None
        #self.totalNumPix=None
        #self.classPercentage=None

    def pubImage(self,img):
        msg = Image()
        msg.header.stamp = rospy.Time.now()
        msg.encoding = 'bgr8'
        msg.height = img.shape[0]
        msg.width = img.shape[1]
        msg.step = img.shape[1] * 3
        msg.data = img.tostring()
        self.imgPub.publish(msg)

    def ImgCallback(self,data):
        try:
            self.inputImg = self.bridge.imgmsg_to_cv2(data, "bgr8")
            if self.outputImg is not None:
                self.pubImage(np.uint8(self.outputImg*255))
            #self.processImg(inputImg)
        except CvBridgeError as e:
            print(e)
    
    def processImgCallback(self, req):
        print 'Processing image...'

        #input_image=self.imgQ.get() #Blocking call??
        input_image = self.inputImg
        #input_image = cv2.resize(input_image, (self.input_shape[3],self.input_shape[2]))
        #imgBGR=cv2.cvtColor(input_image, cv2.COLOR_RGB2BGR) #For opencv visualization
        input_image = input_image.transpose((2,0,1))
        #input_image = input_image[(2,1,0),:,:] # May be required, if you do not open your data with opencv
        input_image = np.asarray([input_image])

        start = time.time()
        #out = net.forward_all(data=input_image)
        self.net.forward(data=input_image)
        end = time.time()
        print '%30s' % 'Executed SegNet in ', str((end - start)*1000), 'ms'

        start = time.time()
        predicted = self.net.blobs['prob'].data
        output = np.squeeze(predicted[0,:,:,:])
        #ind = np.argmax(output, axis=0)

        #r = ind.copy()
        #g = ind.copy()
        #b = ind.copy()
        r = output[2,:,:].copy()
        g = output[1,:,:].copy()
        b = output[0,:,:].copy()
        #self.classPercentage=[np.float(len(r[ind==0]))/np.float(self.totalNumPix),np.float(len(g[ind==1]))/np.float(self.totalNumPix),np.float(len(b[ind==2]))/np.float(self.totalNumPix)]
        #print 'Class percentage: \nbg={}%, crop={}%, weed={}%'.format(self.classPercentage[0]*100,self.classPercentage[1]*100,self.classPercentage[2]*100)
        
        #print self.classPercentage[2]/self.classPercentage[1]
        #tempArray=Float32MultiArray(data=self.classPercentage)
        #self.clsPercPub.publish(tempArray)

        #for l in range(0,3):
        #    r[ind==l] = self.label_colours[l,0]
        #    g[ind==l] = self.label_colours[l,1]
        #    b[ind==l] = self.label_colours[l,2]

        #segmentation_rgb = np.zeros((ind.shape[0], ind.shape[1], 3))
        segmentation_rgb = np.zeros((r.shape[0], r.shape[1], 3))

        #segmentation_rgb[:,:,0] = r/255.0
        #segmentation_rgb[:,:,1] = g/255.0
        #segmentation_rgb[:,:,2] = b/255.0
        segmentation_rgb[:,:,0] = r
        segmentation_rgb[:,:,1] = g
        segmentation_rgb[:,:,2] = b
        
        end = time.time()
        print '%30s' % 'Processed results in ', str((end - start)*1000), 'ms\n'

        segmentation_rgb = segmentation_rgb[:,:,(2,1,0)] #BGR (opencv) to RGB swap
        self.outputImg = segmentation_rgb
        #self.pubImage(np.uint8(segmentation_rgb*255))

        return std_srvs.srv.EmptyResponse()

    def init(self):
        #ROS related things
        rospy.init_node('pySegNet')

        #SegNet related things
        self.net = caffe.Net(self.model,
                self.weights,
                caffe.TEST)
        self.input_shape = self.net.blobs['data'].data.shape
        self.output_shape = self.net.blobs['prob'].data.shape
        self.imgHeight=self.input_shape[2]
        self.imgWidth=self.input_shape[3]
        self.totalNumPix=self.imgHeight*self.imgWidth
        caffe.set_mode_cpu()

        #Mapping related things
    	rit18_val = sio.loadmat('rit18-val.mat')
    	print rit18_val


if __name__ == "__main__":
        mySegNet = segNet();
        mySegNet.init()
        rospy.spin()
        #while not rospy.is_shutdown():
            #mySegNet.processImg()