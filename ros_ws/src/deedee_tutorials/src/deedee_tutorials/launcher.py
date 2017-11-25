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
                        "autonomous": "true",
                        "sim_display": "none"}
        self.cmd = ""

        self.retrieve_config()
        self.build_cmd()
        print self.cmd
        self.spawn()

    def retrieve_config(self):
        for setting in self.configs.keys():
            self.configs[setting] = rospy.get_param("/{}".format(setting))

    def build_cmd(self):
        self.cmd = "roslaunch deedee_tutorials follow_waypoints.launch"
        for setting in self.configs.keys():
            if setting=="sim_display":
                if self.configs[setting] in ["gzclient", "both"]:
                  self.cmd += " gui:=true headless:=false"
                else:
                  self.cmd += " gui:=false headless:=true"
            else:
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
        self.configs = {"sim_display": "none",
                        "gzweb_path": ""}

        self.retrieve_config()
        if self.configs["sim_display"] in ["gzweb", "both"]:
            self.cmd = "{}/start_gzweb.sh".format(self.configs["gzweb_path"])
            subprocess.call("{}/start_gzweb.sh".format(self.configs["gzweb_path"]),
                            shell=True)
            rospy.on_shutdown(self.shutdown_hook)
            rospy.spin()

    def retrieve_config(self):
        for setting in self.configs.keys():
            self.configs[setting] = rospy.get_param("/{}".format(setting))

    def shutdown_hook(self):
        print "Stopping webserver!"
        subprocess.call("{}/stop_gzweb.sh".format(self.configs["gzweb_path"]),
                        shell=True)

