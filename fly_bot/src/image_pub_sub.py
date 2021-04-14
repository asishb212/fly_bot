#!/usr/bin/env python

import rospy
import cv2
from std_msgs.msg import String,Header
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pcl2
from geometry_msgs.msg import Point32
from cv_bridge import CvBridge, CvBridgeError
import sys
i = 0
b = 0.001
f = 0.1
bridge = CvBridge()

def image_callback(ros_image):
    global bridge, i
    # convert ros_image into an opencv-compatible image
    try:
        cv_image = bridge.imgmsg_to_cv2(ros_image, "bgr8")
    except CvBridgeError as e:
        print(e)
    # from now on, you can work exactly like with opencv

    orb = cv2.ORB_create(nfeatures=1500)
    key_points, description = orb.detectAndCompute(cv_image, None)
    frame_keypoints = cv2.drawKeypoints(
        cv_image, key_points, cv_image, flags=0)
    i += 1
    prev_frame = cv_image
    if (i > 1):
        # initializing the BF matcher using NORM_HAMMING
        bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
        key_points1, description1 = orb.detectAndCompute(prev_frame, None)
        matches = bf.match(description, description1)
        #after_matching = cv2.drawMatches(cv_image,key_points,prev_frame,key_points1,matches,None)
        # cv2.imshow("after_matching",after_matching)
        points=[]
        for match in matches:
            p1 = key_points[match.queryIdx].pt
            p2 = key_points1[match.trainIdx].pt
            if p1[0]-p2[0]==0 or p1[1]-p2[1]==0:
              continue
            X=b*p1[0]/(p1[0]-p2[0])
            Y=b*p1[1]/(p1[1]-p2[1])
            Z=f*b/(p1[0]-p2[0])
            points.append([X,Y,Z])
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = 'camera_link'
        #create pcl from points
        scaled_polygon_pcl = pcl2.create_cloud_xyz32(header,points)
        pcl_pub.publish(scaled_polygon_pcl)
        cv2.waitKey(2)

def main(args):
    global pcl_pub
    rospy.init_node('image_converter', anonymous=True)
    image_sub = rospy.Subscriber("/Kwad/camera1/image_raw",Image, image_callback)
    #image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, image_callback)
    pcl_pub = rospy.Publisher("/cloud_in", PointCloud2,queue_size=10)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
