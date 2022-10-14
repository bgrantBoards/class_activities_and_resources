import binascii
import rclpy
from threading import Thread
from rclpy.node import Node
import time
from sensor_msgs.msg import Image
from copy import deepcopy
from cv_bridge import CvBridge
import cv2
import numpy as np
from geometry_msgs.msg import Twist, Vector3

# import installed libraries
from simple_pid import PID
# from vector_2d import Vector, VectorPolar

class BallTracker(Node):
    """ The BallTracker is a Python object that encompasses a ROS node 
        that can process images from the camera and search for a ball within.
        The node will issue motor commands to move forward while keeping
        the ball in the center of the camera's field of view. """

    def __init__(self, image_topic):
        """ Initialize the ball tracker """
        super().__init__('ball_tracker')
        self.cv_image = None                        # the latest image from the camera
        self.bridge = CvBridge()                    # used to convert ROS messages to OpenCV

        self.create_subscription(Image, image_topic, self.process_image, 10)
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        thread = Thread(target=self.loop_wrapper)
        thread.start()

        # image processing params
        self.red_lower_thresh = 0    # red
        self.red_upper_thresh = 10
        self.gre_lower_thresh = 75   # green
        self.gre_upper_thresh = 255
        self.blu_lower_thresh = 0    # blue
        self.blu_upper_thresh = 66

        # follower parameters
        # self.approach_threshold = 1.1 # (m) dist to keep from object
        # self.lin_pid    = PID(Kp=0.1, Ki=0, Kd=0.05, setpoint=0)
        self.steer_pid  = PID(Kp=0.1, Ki=0, Kd=0.05, setpoint=0)    # PID controller for steering

    def process_image(self, msg):
        """ Process image messages from ROS and stash them in an attribute
            called cv_image for subsequent processing """
        self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    def loop_wrapper(self):
        """ This function takes care of calling the run_loop function repeatedly.
            We are using a separate thread to run the loop_wrapper to work around
            issues with single threaded executors in ROS2 """
        cv2.namedWindow('video_window')
        cv2.namedWindow('binary_window')
        cv2.namedWindow('image_info')
        cv2.setMouseCallback('video_window', self.process_mouse_event)

        # create sliders with callbacks to change threshold members
        cv2.createTrackbar('red lower bound', 'binary_window', self.red_lower_thresh, self.red_upper_thresh, self.set_red_lower)
        cv2.createTrackbar('red upper bound', 'binary_window', self.red_upper_thresh, 255, self.set_red_upper)
        cv2.createTrackbar('green lower bound', 'binary_window', self.gre_lower_thresh, 255, self.set_gre_lower)
        cv2.createTrackbar('green upper bound', 'binary_window', self.gre_upper_thresh, 255, self.set_gre_upper)
        cv2.createTrackbar('blue lower bound', 'binary_window', self.blu_lower_thresh, 255, self.set_blu_upper)
        cv2.createTrackbar('blue upper bound', 'binary_window', self.blu_upper_thresh, 255, self.set_blu_upper)
        
        # mouse callback
        cv2.setMouseCallback('video_window', self.process_mouse_event)

        # initiate running loop
        while True:
            self.run_loop()
            time.sleep(0.1)
    
    def set_red_upper(self, val):
        self.red_upper_thresh = val
    
    def set_red_lower(self, val):
        self.red_lower_thresh = val
    
    def set_gre_upper(self, val):
        self.gre_upper_thresh = val
    
    def set_gre_lower(self, val):
        self.gre_lower_thresh = val
    
    def set_blu_upper(self, val):
        self.blu_upper_thresh = val
    
    def set_blu_lower(self, val):
        self.blu_lower_thresh = val

    def process_mouse_event(self, event, x,y,flags,param):
        """ Process mouse events so that you can see the color values
            associated with a particular pixel in the camera images """
        self.image_info_window = 255*np.ones((500,500,3))
        cv2.putText(self.image_info_window,
                    'Color (b=%d,g=%d,r=%d)' % (self.cv_image[y,x,0], self.cv_image[y,x,1], self.cv_image[y,x,2]),
                    (5,50),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1,
                    (0,0,0))

    def run_loop(self):
        # NOTE: only do cv2.imshow and cv2.waitKey in this function
        if not self.cv_image is None:
            lower_thresholds = (self.red_lower_thresh, self.gre_lower_thresh, self.blu_lower_thresh)
            upper_thresholds = (self.red_upper_thresh, self.gre_upper_thresh, self.blu_upper_thresh)
            self.binary_image = cv2.inRange(self.cv_image, lower_thresholds, upper_thresholds)

            # find center of white blob in binary image
            if 255 in self.binary_image:
                locations = np.where(self.binary_image == 255)
                y = np.mean(locations[0])
                x = np.mean(locations[1])
                print(x, y)

            #print(self.cv_image.shape)
            cv2.imshow('video_window', self.cv_image)
            cv2.imshow('binary_window', self.binary_image)
            if hasattr(self, 'image_info_window'):
                cv2.imshow('image_info', self.image_info_window)
            cv2.waitKey(5)

    def command_motors(self, x, y):
        drive_msg = Twist()

        # use PID control to slow down approach to target
        drive_msg.linear.x = 0.1


        if x > (self.cv_image.shape[1]/2):
            drive_msg.angular.z = -1.0
        elif x < (self.cv_image.shape[1]/2):
            drive_msg.angular.z = -1.0
        else:
            drive_msg.angular.z = 0

        self.pub.publish(drive_msg)





if __name__ == '__main__':
    node = BallTracker("/camera/image_raw")
    node.run()


def main(args=None):
    rclpy.init()
    n = BallTracker("camera/image_raw")
    rclpy.spin(n)
    rclpy.shutdown()


if __name__ == '__main__':
    main()