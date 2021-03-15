#!/usr/bin/python
# Inkyu Sa, enddl22@gmail.com
# Marija Popovic, mpopovic514@gmail.com
# June-July 2018
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
import skimage
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String, Int32, Int32MultiArray, MultiArrayLayout, MultiArrayDimension, Float32MultiArray

sys.path.append('/usr/local/lib/python2.7/site-packages')
# Make sure that caffe is on the python path:
caffe_root = '/home/masha/catkin_ws/src/weedNet-devel/caffe-segnet-cudnn5/'
sys.path.insert(0, caffe_root + 'python')
import caffe


class segNet(object):
    def __init__(self):
        self.odom_sub = rospy.Subscriber("odometry", Odometry, self.odomCallback)
        self.img_pub = rospy.Publisher("image_seg", Image, queue_size=1)
        self.process_img_srv = rospy.Service("process_image", std_srvs.srv.Empty, self.processImgCallback)
        self.model = '/home/masha/catkin_ws/src/weedNet-devel/SegNet-Tutorial/Models/segnet_ipp_rit18_inference_live.prototxt'
        self.weights = '/home/masha/catkin_ws/src/weedNet-devel/SegNet-Tutorial/Models/Inference/rit18_weights.caffemodel'
        self.net = None
        self.input_shape = None
        self.img_width = None
        self.img_height = None
        self.img_counter = 0
        self.output_shape = None
        self.FoV_hor = math.radians(47.2)
        self.FoV_ver = math.radians(35.4)

        #  Mapping
        self.ortho_data = None
        self.ortho_labels = None
        self.ortho_GSD = None
        self.mav_pose = None

    def pubImage(self, img):
        msg = Image()
        msg.header.stamp = rospy.Time.now()
        msg.encoding = 'bgr8'
        msg.height = img.shape[0]
        msg.width = img.shape[1]
        msg.step = img.shape[1] * 3
        msg.data = img.tostring()
        self.img_pub.publish(msg)

    def odomCallback(self, data):
        self.mav_pose = data.pose.pose

    def processImgCallback(self, req):
        print 'Processing images...'

        # Crop orthomosaic to get image inputs.
        mav_position = self.mav_pose.position
        [img_size_y, img_size_x] = self.getImgSize(mav_position.z)
        img_position = self.getImgPosition(mav_position)

        input_image_rgb = self.ortho_data[int(img_position.y - (img_size_y / 2)):int(img_position.y + (img_size_y / 2)),
                          int(img_position.x - (img_size_x / 2)):int(img_position.x + (img_size_x / 2)), 3:6]
        input_image_rgb = skimage.transform.resize(input_image_rgb, [480, 360], order=0)
        input_image_rgb = skimage.transform.rotate(input_image_rgb, 90.0, resize=True)
        skimage.io.imsave(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                       "test", "image_rgb%04i.png" % self.img_counter), input_image_rgb)

        input_image_ir1 = self.ortho_data[int(img_position.y - (img_size_y / 2)):int(img_position.y + (img_size_y / 2)),
                          int(img_position.x - (img_size_x / 2)):int(img_position.x + (img_size_x / 2)), 0]
        input_image_ir1 = skimage.transform.resize(input_image_ir1, [480, 360], order=0)
        input_image_ir1 = skimage.transform.rotate(input_image_ir1, 90.0, resize=True)
        skimage.io.imsave(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                       "test", "image_ir1%04i.png" % self.img_counter), input_image_ir1)

        input_image_ir2 = self.ortho_data[int(img_position.y - (img_size_y / 2)):int(img_position.y + (img_size_y / 2)),
                          int(img_position.x - (img_size_x / 2)):int(img_position.x + (img_size_x / 2)), 1]
        input_image_ir2 = skimage.transform.resize(input_image_ir2, [480, 360], order=0)
        input_image_ir2 = skimage.transform.rotate(input_image_ir2, 90.0, resize=True)
        skimage.io.imsave(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                       "test", "image_ir2%04i.png" % self.img_counter), input_image_ir2)

        input_image_ir3 = self.ortho_data[int(img_position.y - (img_size_y / 2)):int(img_position.y + (img_size_y / 2)),
                          int(img_position.x - (img_size_x / 2)):int(img_position.x + (img_size_x / 2)), 2]
        input_image_ir3 = skimage.transform.resize(input_image_ir3, [480, 360], order=0)
        input_image_ir3 = skimage.transform.rotate(input_image_ir3, 90.0, resize=True)
        skimage.io.imsave(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                       "test", "image_ir3%04i.png" % self.img_counter), input_image_ir3)

        # Convert RGB to BGR.
        input_image_rgb = input_image_rgb[:, :, [2, 1, 0]]
        input_image_rgb = input_image_rgb.transpose((2, 0, 1))
        input_image_ir1 = np.asarray([input_image_ir1])
        input_image_ir2 = np.asarray([input_image_ir2])
        input_image_ir3 = np.asarray([input_image_ir3])
        input_image_rgb = np.asarray([input_image_rgb])

        # Forward pass through network.
        start = time.time()
        self.net.forward(dataIR1=input_image_ir1 * 255, dataIR2=input_image_ir2 * 255,
                         dataIR3=input_image_ir3 * 255, dataRGB=input_image_rgb * 255)
        end = time.time()
        print '%30s' % 'Executed SegNet in ', str((end - start) * 1000), 'ms'

        # Result processing.
        start = time.time()
        predicted = self.net.blobs['prob'].data
        output = np.squeeze(predicted[0, :, :, :])

        r = output[17, :, :].copy()
        g = output[1, :, :].copy() + output[4, :, :].copy() + output[3, :, :].copy()
        b = np.ones(r.shape) - r - g

        # Debugging.
        ind = np.argmax(output, axis=0)
        scipy.misc.toimage(ind, low=0.0, high=output.shape[0] - 1).save(
            os.path.join(os.path.dirname(os.path.abspath(__file__)),
                         "test", "image_out_max%04i.png" % self.img_counter))

        # BGR order.
        segmentation_rgb = np.zeros((r.shape[0], r.shape[1], 3))
        segmentation_rgb[:, :, 0] = b
        segmentation_rgb[:, :, 1] = g
        segmentation_rgb[:, :, 2] = r
        end = time.time()
        print '%30s' % 'Processed results in ', str((end - start) * 1000), 'ms\n'

        skimage.io.imsave(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                       "test", "image_out%04i.png" % self.img_counter),
                          np.uint8(segmentation_rgb * 255))
        self.pubImage(np.uint8(segmentation_rgb * 255))

        self.img_counter += 1

        return std_srvs.srv.EmptyResponse()

    def getImgSize(self, altitude):
        img_size_y = round((2 * altitude * math.tan(self.FoV_hor / 2)) / self.ortho_GSD);
        img_size_x = round((2 * altitude * math.tan(self.FoV_ver / 2)) / self.ortho_GSD);
        return img_size_y, img_size_x

    def getImgPosition(self, mav_position):
        img_position = mav_position
        img_position.x = mav_position.x / self.ortho_GSD + self.ortho_data.shape[1] / 2
        img_position.y = -mav_position.y / self.ortho_GSD + self.ortho_data.shape[0] / 2
        return img_position

    def init(self):
        # ROS related things
        rospy.init_node('pySegNet')

        # SegNet related things
        self.net = caffe.Net(self.model, self.weights, caffe.TEST)
        self.input_shape = self.net.blobs['data'].data.shape
        self.output_shape = self.net.blobs['prob'].data.shape
        self.img_height = self.input_shape[2]
        self.img_width = self.input_shape[3]
        caffe.set_mode_cpu()

        # Mapping related things
        ortho = sio.loadmat('/home/masha/catkin_ws/src/mav_ipp/mav_ipp_sim/src/rit18-val.mat', mat_dtype=True)
        self.ortho_data = ortho['val_data_cropped']
        self.ortho_labels = ortho['val_labels_cropped']
        self.ortho_GSD = ortho['GSD']


if __name__ == "__main__":
    mySegNet = segNet();
    mySegNet.init()
    rospy.spin()
# while not rospy.is_shutdown():
# mySegNet.processImg()