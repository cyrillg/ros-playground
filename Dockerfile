FROM ubuntu:16.04
MAINTAINER Cyrill Guillemot "https://github.com/cyrillg"

# Install ROS
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu xenial main" > /etc/apt/sources.list.d/ros-latest.list'
RUN apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
RUN apt-get update && apt-get install -y ros-kinetic-desktop-full
RUN rosdep init

# Install other utilities
ENV DEBIAN_FRONTEND noninteractive

RUN apt-get update && \
    apt-get install -y --no-install-recommends ubuntu-desktop && \
    apt-get install -y gnome-panel \
      gnome-settings-daemon \
      metacity \
      nautilus \
      gnome-terminal \
      x11vnc \
      xvfb \
      openssh-server \
      vim \
      tmux \
      git \
      locales \
      sudo \
      apt-utils \
      supervisor

# Set the locale
RUN sed -i -e 's/# en_US.UTF-8 UTF-8/en_US.UTF-8 UTF-8/' /etc/locale.gen && \
    locale-gen

# Configure ssh
RUN mkdir /var/run/sshd
RUN sed -ri 's/UsePAM yes/#UsePAM yes/g' /etc/ssh/sshd_config

# Environment variables
ENV DISPLAY :1
ENV USER serial
ENV HOME /home/$USER

# Configure user
RUN groupadd $USER && \
    useradd --create-home --no-log-init -g $USER $USER && \
    usermod -aG sudo $USER
RUN echo "$USER:$USER" | chpasswd
RUN chsh -s /bin/bash $USER

# Add files
WORKDIR $HOME
RUN mkdir ros_ws
RUN mkdir ./.gazebo
RUN mkdir -p ./.config/nautilus
RUN mkdir -p /var/log/supervisor
RUN rm .bashrc
RUN curl https://raw.githubusercontent.com/git/git/master/contrib/completion/git-prompt.sh > .git-prompt.bash
COPY files/.bashrc .
COPY files/.tmux.conf .
COPY files/.vimrc .
COPY files/autumn.jpg .
COPY files/supervisord.conf /etc/supervisor/conf.d/supervisord.conf
COPY files/.gazebo ./.gazebo

# Set user and group ownership
RUN chown -R serial:serial .bashrc \
                           .tmux.conf \
                           .vimrc \
                           .gazebo \
                           ros_ws

# TEMPORARY - SHOULD ULTIMATELY BE HANDLED BY ROSDEP
RUN apt-get install -y\
        ros-kinetic-ros-control \
        ros-kinetic-ros-controllers \
        ros-kinetic-gazebo-ros-control

# Install ROS dependencies
WORKDIR $HOME/ros_ws
USER serial
RUN echo "export WS=~/" >> $HOME/.bashrc
RUN rosdep update

EXPOSE 22 5900
USER root

CMD    ["/usr/bin/supervisord"]

