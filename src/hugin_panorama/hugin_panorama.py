#!/usr/bin/env python
import os
import rospy
import shutil
import rospkg
import subprocess
import cv2

from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
from std_srvs.srv import Empty

def load_image(filename):
  img = cv2.imread(filename, -1)
  if img is None:
    rospy.logerr("ERROR image file didn't load or doesn't exist: %s" % filename)
    exit(1)
  return img

class HuginPanorama():
  def __init__(self):
    rospy.Service('stitch', Empty, self.stitch_callback)
    rospy.Service('reset', Empty,  self.reset_callback)
    self.publisher = rospy.Publisher('panorama/compressed', CompressedImage, queue_size=10)

    # Get the path to where input and output images for the panorama are stored
    self.images_path = rospy.get_param('~images_path',self.default_images_path())

    # Create path if it does not exist
    if not os.path.exists(self.images_path):
        os.makedirs(self.images_path)

  def default_images_path(self):
    """By default use 'images/'folder in hugin_panorama package directory

    This is where input and output images for the panorama are stored.
    """
    rospack = rospkg.RosPack()
    return "%s/images/" % rospack.get_path('hugin_panorama')

  def stitch_callback(self, data):
    self.do_stitch()
    return []

  def reset_callback(self, data):
    rospy.loginfo('Clearing images waiting to be stitched.')
    self.cleanup(all=True)
    return []

  def get_image_filenames(self):
    """ returns space seperated list of files in the images_path """
    print(" ".join(os.listdir(self.images_path)))
    return " ".join(os.listdir(self.images_path))

  def hugin_stitch(self):
    files = self.get_image_filenames()
    rospy.loginfo("Stitching files: %s" % files)
    
    # create project file
    self.bash_exec('pto_gen -o pano.pto %s' % files)
    # do cpfind
    self.bash_exec('cpfind -o pano.pto --multirow --celeste pano.pto')
    # do clean
    self.bash_exec('cpclean -o pano.pto pano.pto')
    # do vertical lines
    self.bash_exec('linefind -o pano.pto pano.pto')
    # do optimize locations
    self.bash_exec('autooptimiser -a -m -l -s -o pano.pto pano.pto')
    # calculate size
    self.bash_exec('pano_modify --canvas=AUTO --crop=AUTO -o pano.pto pano.pto')
    # stitch
    self.bash_exec('hugin_executor --stitching --prefix=output pano.pto')
    # compress
    self.bash_exec('convert output.tif output.png')

    rospy.loginfo('Panorama saved to: %soutput.png' % self.images_path)

  def publish_pano(self):
    img = load_image(self.images_path + 'output.png')
    compressed_image = CvBridge().cv2_to_compressed_imgmsg(img)
    self.publisher.publish(compressed_image)

  def cleanup(self, all=False):
    if all:
      # Delete and recreate the images_path
      shutil.rmtree(self.images_path)
      os.makedirs(self.images_path)
    else:
      # Deletes previous hugin project files and output only
      if os.path.isfile(self.images_path + 'output.tif'):
        os.remove(self.images_path + 'output.tif')
      if os.path.isfile(self.images_path + 'output.png'):
        os.remove(self.images_path + 'output.png')
      if os.path.isfile(self.images_path + 'pano.pto'):
        os.remove(self.images_path + 'pano.pto')

  def do_stitch(self):
    rospy.loginfo('Stitching panorama...')
    self.cleanup()
    self.hugin_stitch()
    self.publish_pano()
    rospy.loginfo('Finished.')

  def bash_exec(self, cmd):
    sp = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, cwd=self.images_path)
    out, err = sp.communicate()
    if out:
        rospy.loginfo("\n%s" % out)
    if err:
        rospy.loginfo("\n%s" % err)
    return out, err, sp.returncode

def main():
  rospy.init_node('hugin_panorama')
  Pano = HuginPanorama()
  rospy.spin()
