#!/usr/bin/env python

# TODO PerceptionNode (PseudoDepthImage)
import rospy
from std_msgs.msg import String, Bool, Empty, Float32MultiArray, MultiArrayDimension
import numpy as np

def PseudoDepthImage() -> None:
  pub = rospy.Publisher('proc_output_depth_image', Float32MultiArray, queue_size=10)
  rospy.init_node('DepthImage', anonymous=True)
  rate = rospy.Rate(10) # 10hz
  while not rospy.is_shutdown():
    z_input = input('(z): ')
    z = float(z_input)
    depth_image : Float32MultiArray = Float32MultiArray()
    depth_image.layout.dim = [MultiArrayDimension(), MultiArrayDimension(), MultiArrayDimension()]
    depth_image.layout.dim[0].label  = "height"
    depth_image.layout.dim[0].size   = 480
    depth_image.layout.dim[0].stride = 1*640*480
    depth_image.layout.dim[1].label  = "width"
    depth_image.layout.dim[1].size   = 640
    depth_image.layout.dim[1].stride = 1*640
    depth_image.layout.dim[2].label  = "channel"
    depth_image.layout.dim[2].size   = 1
    depth_image.layout.dim[2].stride = 1
    #
    depth_image_size = depth_image.layout.dim[0].stride
    depth_image.data = [z]*depth_image_size
    pub.publish(depth_image)

if __name__ == '__main__':
  try:
    PseudoDepthImage()
  except rospy.ROSInterruptException:
    pass