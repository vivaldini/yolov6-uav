#!/usr/bin/python3
import sys
import rospy
from std_msgs.msg import String, Bool # For pub/sub
from std_msgs.msg import Header
from geometry_msgs.msg import Pose,PoseArray
import numpy as np
from tf.transformations import euler_from_quaternion, rotation_matrix, concatenate_matrices
import math
import pandas as pd

from sklearn.cluster import KMeans

class image_converter:

    def __init__(self):
        # self.image_pub = rospy.Publisher("/target_image",Image, queue_size=1)
        self.object_coordinates_pub = rospy.Publisher('/objects_point_1', PoseArray, queue_size=1)
        
        # self.bridge = CvBridge()
        rospy.init_node('tracking', anonymous=True)

        # self.image_sub = rospy.Subscriber("/camera/image/compressed",CompressedImage,self.callback)
        self.object_coordinates_sub = rospy.Subscriber('/object_coordinates', PoseArray, self.callback)
        self.coverage_status_sub = rospy.Subscriber("/coverage_status", Bool, self.coverage_status_callback)
        self.df = pd.DataFrame(columns=['x', 'y', 'dtime' 'score'])

        self.time = 0
        self.coverage_status = False
        self.publish_data = False
    
    def coverage_status_callback(self, data):
        self.coverage_status = data.data

    def callback(self,data):
        if self.time == 0:
            self.time = data.header.stamp.secs
        # iterate over Pose array
        for pose in data.poses:
            # self.df = self.df.append({'x': pose.position.x,
            #                           'y': pose.position.y,
            #                           'score': pose.orientation.x,
            #                           'dtime': data.header.stamp.secs - self.time}, ignore_index=True)
             self.df = self.df.append({'x': pose.position.x,
                                      'y': pose.position.y,
                                      'score': pose.orientation.x,
                                      'dtime': data.header.stamp.secs - self.time}, ignore_index=True)       
        if self.coverage_status is True:
            print ("Publishing identified objects")
            self.get_objects_positions()
            self.publish_data = True

            # Reset
            self.coverage_status = False
            self.df = pd.DataFrame(columns=['x', 'y', 'dtime' ,'score'])

        if self.publish_data is True:
            self.object_coordinates_pub.publish(self.msg)
            
    
    def get_objects_positions(self):
        kmeans = KMeans(n_clusters=5, random_state=0).fit(self.df[['x', 'y']])

        # get cluster centers
        centers = kmeans.cluster_centers_

        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "map"
        self.msg = PoseArray()

        for center in centers:
            pose = Pose()
            pose.position.x = center[0]
            pose.position.y = center[1]

            self.msg.poses.append(pose)
    
    def return_data(self):
        return self.df

def main(args):
    ic = image_converter()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    df = ic.return_data()
    # Save on absolute path
    df.to_csv('tracking.csv', index=False)

if __name__ == '__main__':
    main(sys.argv)