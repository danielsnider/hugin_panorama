#!/usr/bin/env python
import time
import rospy

from std_srvs.srv import Empty

if __name__ == '__main__':
  rospy.init_node('stitch_loop')
  rospy.wait_for_service('/hugin_panorama/image_saver1/save')
  rospy.wait_for_service('/hugin_panorama/image_saver2/save')
  rospy.wait_for_service('/hugin_panorama/stitch')
  rospy.wait_for_service('/hugin_panorama/reset')
  save_image1 = rospy.ServiceProxy('/hugin_panorama/image_saver1/save', Empty)
  save_image2 = rospy.ServiceProxy('/hugin_panorama/image_saver2/save', Empty)
  stitch = rospy.ServiceProxy('/hugin_panorama/stitch', Empty)
  reset = rospy.ServiceProxy('/hugin_panorama/reset', Empty)

  while not rospy.is_shutdown():
    rospy.loginfo("Clearing previous panorama files")
    reset()
    rospy.loginfo("Saving image from camera #1")
    save_image1()
    rospy.loginfo("Saving image from camera #2")
    save_image2()
    time.sleep(1) # wait for image files to be written to disk
    rospy.loginfo("Stitching panorama...")
    stitch()
    rospy.loginfo("Done. Looping...")
