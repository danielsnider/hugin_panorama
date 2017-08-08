#!/usr/bin/env python
import os
import glob
import shutil
import subprocess
import cv2
import rospy
import rospkg

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
    self.images_path = rospy.get_param('~images_path')

    # Create path if it does not exist
    if not os.path.exists(self.images_path):
        os.makedirs(self.images_path)

    rospy.loginfo('Using working directory: %s'% self.images_path)

  def stitch_callback(self, data):
    self.do_stitch()
    return []

  def reset_callback(self, data):
    rospy.loginfo('Clearing images waiting to be stitched.')
    self.cleanup(delete_images=True)
    return []

  def get_image_filenames(self):
    """ returns space seperated list of files in the images_path """
    print(" ".join(os.listdir(self.images_path)))
    return " ".join(os.listdir(self.images_path))

  def hugin_stitch(self):
    files = self.get_image_filenames()

    if files == None or files == "":
      rospy.logerr("Did not find any images to stitch in: %s" % self.images_path)
      return

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

    output_file = os.path.join(self.images_path, 'output.png')
    if not os.path.isfile(output_file):
      rospy.logerr('Hugin failed to create a panorama.')
      return

    rospy.loginfo('Panorama saved to: %s/output.png' % self.images_path)

    self.publish_pano()
    rospy.loginfo('Finished.')

  def publish_pano(self):
    img = load_image(os.path.join(self.images_path, 'output.png'))
    compressed_image = CvBridge().cv2_to_compressed_imgmsg(img)
    self.publisher.publish(compressed_image)

  def cleanup(self, delete_images=False):
    # Hugin project files and output images
    files_to_be_deleted = ['output.tif', 'output.png', 'pano.pto']

    # Optionally delete images
    if delete_images:
      image_types = ('*.png', '*.jpg', '*.gif')
      for file_type in image_types:
        path = os.path.join(self.images_path,file_type)
        files_to_be_deleted.extend(glob.glob(path))

    # Do file deletion
    for file in files_to_be_deleted:
      file_to_be_deleted = os.path.join(self.images_path, file)
      if os.path.isfile(file_to_be_deleted):
        os.remove(file_to_be_deleted)

  def do_stitch(self):
    rospy.loginfo('Stitching panorama...')
    self.cleanup()
    self.hugin_stitch()

  def bash_exec(self, cmd):
    sp = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, cwd=self.images_path)
    out, err = sp.communicate()
    if out:
        rospy.loginfo("\n%s" % out)
    if err:
        rospy.logerr("\n%s" % err)
    return out, err, sp.returncode

def main():
  rospy.init_node('hugin_panorama')
  Pano = HuginPanorama()
  rospy.spin()
