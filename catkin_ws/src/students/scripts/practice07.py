#!/usr/bin/env python
#
# MOBILE ROBOTS - FI-UNAM, 2022-2
# PRACTICE 7 - COLOR SEGMENTATION
#
# Instructions:
# Complete the code to estimate the position of an object 
# given a colored point cloud using color segmentation.
#

import numpy
import cv2
import ros_numpy
import rospy
import math
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PointStamped, Point
from custom_msgs.srv import FindObject, FindObjectResponse

NAME = "Valle Rodriguez Andrea"

def segment_by_color(img_bgr, points, obj_name):

    upperLimit = [0]
    lowerLimit = [0]

    if obj_name == "pringles":
        upperLimit = [35, 255, 255]
        lowerLimit = [25, 50, 50]
    else:
        upperLimit = [20, 255, 255]
        lowerLimit = [10, 200, 50]
   
    img_hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
    img_bin = cv2.inRange(img_hsv, numpy.array(lowerLimit), numpy.array(upperLimit))
    nonZero = cv2.findNonZero(img_bin)
    meanCentroide = cv2.mean(nonZero)

    x, y, z = 0, 0, 0

    for i in nonZero:
        [[c,r]] = i
        if math.isnan(points[r,c][0]) or math.isnan(points[r,c][1]) or math.isnan(points[r,c][2]):
            pass
        else:
            x = x + points[r,c][0]
            y = y + points[r,c][1]
            z = z + points[r,c][2]

    x = x/len(nonZero)
    y = y/len(nonZero)
    z = z/len(nonZero)

    return [meanCentroide[0], meanCentroide[1], x, y, z]

def callback_find_object(req):
    global pub_point, img_bgr
    print("Trying to find object: " + req.name)
    arr = ros_numpy.point_cloud2.pointcloud2_to_array(req.cloud)
    rgb_arr = arr['rgb'].copy()
    rgb_arr.dtype = numpy.uint32
    r,g,b = ((rgb_arr >> 16) & 255), ((rgb_arr >> 8) & 255), (rgb_arr & 255)
    img_bgr = cv2.merge((numpy.asarray(b,dtype='uint8'),numpy.asarray(g,dtype='uint8'),numpy.asarray(r,dtype='uint8')))
    [r, c, x, y, z] = segment_by_color(img_bgr, arr, req.name)
    hdr = Header(frame_id='kinect_link', stamp=rospy.Time.now())
    pub_point.publish(PointStamped(header=hdr, point=Point(x=x, y=y, z=z)))
    cv2.circle(img_bgr, (int(r), int(c)), 20, [0, 255, 0], thickness=3)
    resp = FindObjectResponse()
    resp.x, resp.y, resp.z = x, y, z
    return resp

def main():
    global pub_point, img_bgr
    print "PRACTICE 07 - " + NAME
    rospy.init_node("color_segmentation")
    rospy.Service("/vision/find_object", FindObject, callback_find_object)
    pub_point = rospy.Publisher('/detected_object', PointStamped, queue_size=10)
    img_bgr = numpy.zeros((480, 640, 3), numpy.uint8)
    loop = rospy.Rate(10)
    while not rospy.is_shutdown():
        cv2.imshow("Color Segmentation", img_bgr)
        cv2.waitKey(1)
        loop.sleep()
    
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
