#Kelen Vivaldini
#Diego Soler

#!/usr/bin/python3
import sys
import rospy
from std_msgs.msg import String, Bool

from nav_msgs.msg import Odometry
# from sensor_msgs.msg import CompressedImage # Subscriber
# from sensor_msgs.msg import Image,CameraInfo # Publisher
from sensor_msgs.msg import CameraInfo
from vision_msgs.msg import Detection2DArray
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Pose,PoseArray
import numpy as np
from tf.transformations import euler_from_quaternion
import math

class image_converter:

    def __init__(self):
        # self.image_pub = rospy.Publisher("/target_image",Image, queue_size=1)
        
        # Publish the coordinates of the target
        self.object_coordinates_pub = rospy.Publisher('/object_coordinates', PoseArray, queue_size=1)
        
        # self.bridge = CvBridge()
        rospy.init_node('convert', anonymous=True)

        # Publish the status of the identified object
        self.objident_status = rospy.Publisher("/objident_status", Bool, queue_size=1)

        # self.image_sub = rospy.Subscriber("/camera/image/compressed",CompressedImage,self.callback)
        self.detection_sub = rospy.Subscriber("/yolov6/yolov6", Detection2DArray, self.detection_callback)
        self.coverage_status_sub = rospy.Subscriber("/coverage_status", Bool, self.coverage_status_callback)
        self.odom_sub = rospy.Subscriber("/uav1/odometry/odom_main", Odometry, self.odom_callback)
        #self.camera_info_sub = rospy.Subscriber("/uav1/rgbd/color/camera_info", CameraInfo, self.camera_info_callback)
        self.camera_info_sub = rospy.Subscriber("/uav1/bluefox_optflow/camera_info", CameraInfo, self.camera_info_callback)

        self.K = []
        self.coverage_status = False

        # Rqc -> Rotation matrix - Camera to Quadrotor
        Rqc = [[0, 1, 0],[1, 0, 0],[0, 0, -1]]
        self.Rqc = np.matrix(Rqc)

        #Pqc - Position Vector of Camera in relation to Quadrotor
        Pqc = [[0],[0], [0]] 
        self.Pqc = np.matrix(Pqc)

    def image_callback(self,imagem):
        """  #- yolo:.  #- yolo:ç
  #    layout: tiled
  #    panes:
  #      - waitForOdometry; roslaunch yolo-uav yolov6.launch
  #    layout: tiled
  #    panes:
  #      - waitForOdometry; roslaunch yolo-uav yolov6.launch
        Convert the UAV image to an OpenCV image.
	    Filter the desired box, calculate the center.
        

        
        Publish to two topics
        :param data  sensor_msg/CompressedImage. Image obtained in the topic.
        :return      cv_image. cv::Mat. Image in the OpenCV.
        
        """
        try:
            self.cv_image = self.bridge.compressed_imgmsg_to_cv2(imagem, "bgr8")
        except CvBridgeError as e:
            print(e)

    def coverage_status_callback(self, data):
        self.coverage_status = data.data
    
    def odom_callback(self, data):
        self.odom = data
        self.latitude = data.pose.pose.position.x
        self.longitude = data.pose.pose.position.y
        self.altitude = data.pose.pose.position.z
        
        orientation_list = [data.pose.pose.orientation.x,
                            data.pose.pose.orientation.y,
                            data.pose.pose.orientation.z,
                            data.pose.pose.orientation.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        
        #Pwq - Position Vector of Quadrotor in relation to world
        self.Pwq = np.array([[self.latitude], [self.longitude], [self.altitude]])

        # Rwq -> Rotation matrix - world to Quadrotor
        Rz = [[math.cos(yaw), math.sin(yaw), 0], [-math.sin(yaw), math.cos(yaw), 0], [0, 0, 1]]
        Rz = np.matrix(Rz)
        Ry = [[math.cos(pitch), 0, -math.sin(pitch)], [0, 1, 0], [math.sin(pitch), 0, math.cos(pitch)]]
        Ry = np.matrix(Ry)
        Rx = [[1, 0, 0], [0, math.cos(roll), math.sin(roll)], [0, -math.sin(roll), math.cos(roll)]]
        Rx = np.matrix(Rx)
        
        self.Rwq  = Rx * Ry * Rz

        #Rwc - > Rotation matrix - World to Camera
        self.Rwc = self.Rwq * self.Rqc

        #Pwc - Position Vector of Camera in relation to world
        self.Pwc = self.Rwq * self.Pqc + self.Pwq

    def camera_info_callback(self, data):
        if len(self.K) == 0:
            self.K = np.array(data.K).reshape(3,3)
            print(f'K = {self.K}')
    
    def detection_callback(self, data):
        self.detection = data

        if len(data.detections) > 0:
            # print(f'data.header.seq: {data.header.seq}')
            # print(f'data.header.stamp: {data.header.stamp}')
            # print(f'data.header.frame_id: {data.header.frame_id}')
            
            # print(f'number detections = {len(data.detections)}')

            self.calc_transformation_parameters()
            
            msg = PoseArray()
            msg.header = data.header
            msg.header.frame_id = 'map'

            for detection in data.detections:
                bbox = detection.bbox
                if bbox.size_x > 0 and bbox.size_y > 0:
                    if bbox.center.x > 20 and bbox.center.y > 20: # Cutting box to avoid glare
                        # print(detection.bbox)
                        # print(f'class = {detection.results[0].id}')
                        # print(f'score = {detection.results[0].score}')

                        x,y = self.transform_to_world_coordinates(bbox.center)

                        pose = Pose()
                        pose.position.x = x
                        pose.position.y = y
                        pose.position.z = 0
                        pose.orientation.x = detection.results[0].score

                        msg.poses.append(pose)

                        print(pose)
            
            if len(msg.poses) > 0:
                self.object_coordinates_pub.publish(msg)
    
    def calc_transformation_parameters(self):
        #Transpose - Vectors
        R1 = [[(self.Rwc[0,0])],[self.Rwc[0,1]],[self.Rwc[0,2]]]
        R1 = np.matrix(R1).T
        R2 = [[self.Rwc[1,0]],[self.Rwc[1,1]],[self.Rwc[1,2]]]
        R2 = np.matrix(R2).T
        R3 = [[self.Rwc[2,0]],[self.Rwc[2,1]],[self.Rwc[2,2]]]
        R3 = np.matrix(R3).T

        # C -> Intrisic Camera Matrix
        Cext1 = -R1*self.Pwc
        Cext2 = -R2*self.Pwc
        Cext3 = -R3*self.Pwc
        Cext = [[R1[0,0],R1[0,1],R1[0,2],Cext1[0,0]],[R2[0,0],R2[0,1],R2[0,2],Cext2[0,0]],[R3[0,0],R3[0,1],R3[0,2],Cext3[0,0]]]

        Cext = np.matrix(Cext)
        M = self.K * Cext
        M = np.matrix(M)
        p_Z = 0 # Altura eucalipto
        
        Ox, Oy, Oz = self.K[0,2], self.K[1,2], self.K[2,2]
        fsx, fsy = self.K[0,0], self.K[1,1]
        
        self.a, self.b, self.c = M[0,0], M[0,1], M[0,3] + M[0,2] * p_Z
        self.d, self.e, self.f = M[1,0], M[1,1], M[1,3] + M[1,2] * p_Z
        self.g, self.h, self.i = M[2,0], M[2,1], M[2,3] + M[2,2] * p_Z

    # transform to world coordinates 
    def transform_to_world_coordinates(self, center):
        X_im = center.x
        Y_im = center.y
        y = (self.c*self.d - Y_im*self.c*self.g -X_im*self.d*self.i + X_im*self.f*self.g - self.a*self.f + Y_im*self.a*self.i) / (self.a*self.e - X_im*self.e*self.g - Y_im*self.a*self.h - self.b*self.d + X_im*self.d*self.h + Y_im*self.b*self.g)
        x = ((self.b - X_im*self.h) * y + self.c - X_im*self.i) / (X_im*self.g - self.a)

        return x, y

        



def main(args):
  ic = image_converter()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
#   cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
