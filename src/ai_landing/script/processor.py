#! usr/bin python3
from __future__ import print_function
from re import T

import rospy
import cv2
from rospy.impl.tcpros_base import start_tcpros_server
from std_msgs.msg import String,Float32
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import tensorflow as tf
import numpy as np
import time

import os

physical_devices = tf.config.experimental.list_physical_devices('GPU')
if len(physical_devices) > 0:
    for k in range(len(physical_devices)):
        tf.config.experimental.set_memory_growth(physical_devices[k], True)
        print('memory growth:', tf.config.experimental.get_memory_growth(physical_devices[k]))
    else:
        print("Not enough GPU hardware devices available")

class Model1(tf.keras.Model):
    def __init__(self, width, height):
        super(Model1, self).__init__()
        self.width = width
        self.height = height
        # 图像处理网络
        initializer = tf.keras.initializers.orthogonal()
        self.conv1 = tf.keras.layers.Conv2D(
            filters=8,
            kernel_size=7,
            input_shape=(width, height, 1),
            activation='relu',
            name='conv1',
            kernel_initializer=initializer
        )
        self.pool1 = tf.keras.layers.MaxPooling2D(pool_size=(2, 2), strides=2, name='pool1')
        self.conv2 = tf.keras.layers.Conv2D(filters=16, kernel_size=4, activation='relu', name="conv2",
                                            kernel_initializer=initializer)
        self.pool2 = tf.keras.layers.MaxPooling2D(pool_size=(2, 2), strides=2, name="pool2")
        self.flatten = tf.keras.layers.Flatten()
        self.IMG_layer1 = tf.keras.layers.Dense(128, activation='tanh', name="IMG_layer1",
                                                kernel_initializer=initializer)
        self.IMG_layer2 = tf.keras.layers.Dense(64, activation="tanh", name="IMG_layer2",
                                                kernel_initializer=initializer)

        # 输出层
        self.output_layer1 = tf.keras.layers.Dense(64, activation='tanh', name="output_layer1",
                                                   kernel_initializer=initializer)

        self.vel_1 = tf.keras.layers.Dense(32, activation='tanh', name="vel_1", kernel_initializer=initializer)
        self.vel_2 = tf.keras.layers.Dense(18, activation='tanh', name="vel_2", kernel_initializer=initializer)
        self.vel = tf.keras.layers.Dense(3, activation='tanh', name="vel", kernel_initializer=initializer)

    @tf.function
    def call(self, IMG):
        IMG = self.conv1(IMG)
        IMG = self.pool1(IMG)
        IMG = self.conv2(IMG)
        IMG = self.pool2(IMG)
        IMG = self.flatten(IMG)
        # IMG = tf.expand_dims(IMG, axis=1)
        IMG = self.IMG_layer1(IMG)
        IMG = self.IMG_layer2(IMG)
        out = self.output_layer1(IMG)
        vel = self.vel_1(out)
        vel = self.vel_2(vel)
        vel = self.vel(vel)
        output = tf.concat([vel], axis=1)
        return output

class AILanding:
    def __init__(self):
        self.rate = rospy.Rate(20)

        self.bridge = CvBridge()

        self.image_sub = rospy.Subscriber('usb_cam/image_raw',Image,self.IMGcallback)

        self.vx_pub = rospy.Publisher('velocity_x',Float32,queue_size=10)
        self.vy_pub = rospy.Publisher('velocity_y',Float32,queue_size=10)
        self.vz_pub = rospy.Publisher('velocity_z',Float32,queue_size=10)

        # self.cv_image = None # gray image

        self.model = Model1(64,64)
        IMG = np.random.normal(size=(1, 64, 64, 1))
        self.model(IMG)
        self.model.load_weights('/home/aquatao/CODE/AILanding/src/ai_landing/script/policy.h5')

        time.sleep(5)

    def IMGcallback(self,data):
        # start = time.time()
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data,'bgr8')
            # cv_image = cv2.cvtColor(cv_image,cv2.COLOR_BGR2GRAY)
            cv_image = cv_image[:,:,1]
        except CvBridgeError as e:
            print(e)

        cv_image = cv2.resize(cv_image,(64,64))
        cv_image = np.array(cv_image,dtype=np.float32).reshape(1,64,64,1)
        cv_image = tf.convert_to_tensor(cv_image,dtype=tf.float32)
        velocity = tf.squeeze(self.model(cv_image),axis=0).numpy()*np.array([-1,1,1])
        # end = time.time()
        # print(end-start)
        self.vx_pub.publish(velocity[0])
        self.vy_pub.publish(velocity[1])
        self.vz_pub.publish(velocity[2])
        # self.vx_pub.publish(0)
        # self.vy_pub.publish(0)
        # self.vz_pub.publish(0)
        # print('velocity:{}'.format(velocity))
        # print(velocity)
        self.rate.sleep()
        # self.cv_image = cv_image


if __name__ == "__main__":
    rospy.init_node('AILanding',anonymous=True)

    landing = AILanding()
    rospy.spin()
    

    # while not rospy.is_shutdown():
    #     cv2.imshow('im',landing.cv_image)
    #     cv2.waitKey(3)
        # landing.image_sub
        # rospy.SubscribeListener()

    
