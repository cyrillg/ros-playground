# Welcome to the ROS Playground [![Build Status](https://travis-ci.org/cyrillg/ros-playground.svg?branch=master)](https://travis-ci.org/cyrillg/ros-playground)

This repository contains the source code for a simulated mobile robot using the ROS middleware. It is primarily meant to be used in parallel to the educational website [Serial Robotics](https://serial-robotics.org/) ( /!\ not ready yet /!\ ).

The goal is to experiment on topics like _control_, _localization_, _path planning_, all code being made available here, and explained on the website.

Both are still in their (very) early phases but start to take shape, so stay tuned.

## Setup the playground

If you already have a ROS environment ready and/or want to set it up on your own, you can stop reading here. If it is not the case, tools are available to ease your life.

### sr-cli

This is a Bash command-line interface, and the most specific tool available for Serial Robotics users. It uses docker and is tailored to allow the quick deployment of a ready-to-use environment.

Visit the [GitHub page](https://github.com/cyrillg/sr-cli) for more information and demos.

### sr-dev

This is the docker image that is used under the hood by sr-cli. Using it directly might be better for people who prefer to have full control over what they do.

Visit the [GitHub page](https://github.com/cyrillg/sr-dev) for more information.

### vnc-ros-gnome

Very similar to sr-dev, this is also a docker image. The only difference is in the fact that this one is meant to remain unchanged while sr-dev is bound to grow along with Serial Robotic's content. vnc-ros-gnome is actually the base image for sr-dev.

Visit the [GitHub page](https://github.com/cyrillg/vnc-ros-gnome) for more information.

_I hope this will be useful to you. For any inquiry, you can contact me through the website._
