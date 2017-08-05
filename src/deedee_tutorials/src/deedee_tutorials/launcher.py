#! /usr/bin/env python

import rospy
import subprocess

class MainLauncher:
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

class GzwebManager:
  ''' Node spawning the environment with respect to the global configs
  '''
  def __init__(self):
    rospy.init_node("gzweb_manager")
    rospy.sleep(0.5)

    # Configs
    self.configs = {"gzweb_enable": "true",
                    "gzweb_path": ""}

    self.retrieve_config()
    if self.configs["gzweb_enable"]:
      self.cmd = "{}/start_gzweb.sh".format(self.configs["gzweb_path"])
      subprocess.call("{}/start_gzweb.sh".format(self.configs["gzweb_path"]),
                      shell=True)
      rospy.on_shutdown(self.shutdown_hook)
      rospy.spin()

  def retrieve_config(self):
    gzweb_params = rospy.get_param("gzweb")
    for setting in self.configs.keys():
      self.configs[setting] = gzweb_params[setting]

  def shutdown_hook(self):
    print "Stopping webserver!"
    subprocess.call("{}/stop_gzweb.sh".format(self.configs["gzweb_path"]),
                    shell=True)

