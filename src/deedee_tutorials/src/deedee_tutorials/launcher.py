#! /usr/bin/env python

import rospy
import subprocess

class WorldLauncher:
  ''' Node spawning the environment with respect to the global configs
  '''
  def __init__(self):
    rospy.init_node("middleware_spawner")
    rospy.sleep(0.5)

    # Configs
    self.configs = {"robot_name": "deedee",
                    "sim_plant": "true",
                    "autonomous": "true"}
    self.cmd = ""

    self.retrieve_config()
    self.build_cmd()
    print "\n\n\n",self.cmd,"\n\n\n"
    self.spawn()

  def retrieve_config(self):
    for setting in self.configs.keys():
      self.configs[setting] = rospy.get_param("/{}".format(setting))

  def build_cmd(self):
    self.cmd = "roslaunch deedee_tutorials follow_waypoints.launch"
    for setting in self.configs.keys():
      self.cmd += " {}:={}".format(setting, self.configs[setting])

  def spawn(self):
    subprocess.call(self.cmd, shell=True)

class WebLauncher:
  ''' Node spawning the environment with respect to the global configs
  '''
  def __init__(self):
    rospy.init_node("web_interface_spawner")
    rospy.sleep(0.5)

    # Configs
    self.configs = {"gzweb_enable": "true",
                    "gzweb_path": ""}

    self.retrieve_config()
    self.cmd = "{}/start_gzweb.sh".format(self.configs["gzweb_path"])
    print "\n\n\n",self.cmd,"\n\n\n"
    if self.configs["gzweb_enable"]:
      self.spawn()

    rospy.on_shutdown(self.shutdown_hook)
    rospy.spin()

  def retrieve_config(self):
    for setting in self.configs.keys():
      self.configs[setting] = rospy.get_param("/{}".format(setting))

  def spawn(self):
    subprocess.call(self.cmd, shell=True)

  def shutdown_hook(self):
    print "Stopping webserver!"
    subprocess.call("{}/stop_gzweb.sh".format(self.configs["gzweb_path"]),
                    shell=True)

