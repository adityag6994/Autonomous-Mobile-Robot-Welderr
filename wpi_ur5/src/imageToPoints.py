#!/usr/bin/env python    
import rospy
from std_msgs.msg import String
import numpy as np
import cv2
from matplotlib import pyplot as plt
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseArray
from cv_bridge import CvBridge, CvBridgeError

class Image_processor:

    def __init__(self):
        #self.pub = rospy.Publisher("percTopic", PoseArray, queue_size=10)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/kinect/depth/image_raw", Image, self.perception)
    
    # Perception Block
    def perception(image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(image, "bgr8")
        except CvBridgeError as e:
            print(e)
        #img = cv2.imread(filename)
        img = np.asarray(cv_image, dtype="int32")
        gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
        
        corners = cv2.goodFeaturesToTrack(gray,25,0.01,10)
        print corners
        
        rospy.loginfo("corners")
        rospy.loginfo(corners)
        #corners = np.int0(corners)
        #print corners
        arr = []
        
        for i in range(4,12):
               arr=np.append(arr,corners[i])
        
        arr = map(int, arr)
        
        #the points we want
        arr = np.reshape(arr,(8,2))
        #pub.publish(arr)
    
def main():
    ip = Image_processor()
    rospy.init_node("imageToPoints", anonymous=True)
    rate = rospy.Rate(10) # 10hz
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()
  
if __name__ == '__main__':
    main()