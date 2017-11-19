#! /usr/bin/env python

import rospy
from std_msgs.msg import Header
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Pose, Point

rospy.init_node("path_sender")
rospy.sleep(2.)

pub = rospy.Publisher("/deedee/path", Path, queue_size=10)

path = Path(poses=[PoseStamped(pose=Pose(position=Point(x=1)),
                               header=Header(stamp=rospy.Time.now()))])
seq = 0

while not rospy.is_shutdown():
  print seq
  pub.publish(path)
  seq += 1
  rospy.sleep(2.)
