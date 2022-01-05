#！ ~/anaconda3/env/rlenv-gpu/bin/python
from logging import shutdown
import rospy
import cv2
from rospy.impl.tcpros_base import start_tcpros_server
from std_msgs.msg import String,Float32
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Image
import tensorflow as tf
import numpy as np
import time
import airsim
import os

# os.environ["CUDA_VISIBLE_DEVICES"] = "-1"
# os.environ["TF_CPP_MIN_LOG_LEVEL"] = "2"
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
        self.rate = rospy.Rate(25)

        self.vx_pub = rospy.Publisher('velocity_x',Float32,queue_size=10)
        self.vy_pub = rospy.Publisher('velocity_y',Float32,queue_size=10)
        self.vz_pub = rospy.Publisher('velocity_z',Float32,queue_size=10)
        print('ok')

        self.model = Model1(64,64)
        IMG = np.random.normal(size=(1, 64, 64, 1))
        self.model(IMG)
        self.model.load_weights('/home/aquatao/CODE/AILanding/src/ai_landing/script/policy.h5')

        self.client = airsim.MultirotorClient()
        self.client.confirmConnection()

    def Mainloop(self):
        responses = self.client.simGetImages([airsim.ImageRequest("l", airsim.ImageType.Scene, False, False)])
        response = responses[0]
        img1d = np.frombuffer(response.image_data_uint8, dtype=np.uint8)
        img_rgb = img1d.reshape(response.height, response.width, 3)
        img_rgb = np.flipud(img_rgb)
        img = img_rgb[:, :, 0]
        img = cv2.resize(img, (64, 64))
        img = np.array(img, dtype=np.float32).reshape((1, 64, 64, 1))

        cv_image = tf.convert_to_tensor(img,dtype=tf.float32)
        velocity = tf.squeeze(self.model(cv_image),axis=0).numpy()*np.array([-1,1,1])
        # velocity = np.array([0,1,0])
        # print(velocity)
        self.vx_pub.publish(velocity[0])
        self.vy_pub.publish(velocity[1])
        self.vz_pub.publish(velocity[2])
        self.rate.sleep()

if __name__ == "__main__":
    rospy.init_node('AILanding',anonymous=True)
    landing = AILanding()
    while not rospy.is_shutdown():
        landing.Mainloop()
        


