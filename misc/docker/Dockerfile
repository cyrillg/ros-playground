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
#RUN sed -ri 's/^PermitRootLogin\s+.*/PermitRootLogin yes/' /etc/ssh/sshd_config
RUN sed -ri 's/UsePAM yes/#UsePAM yes/g' /etc/ssh/sshd_config
EXPOSE 22

# Configure user
ENV USERNAME serial
ENV HOME /home/$USERNAME

RUN groupadd serial && \
    useradd --create-home --no-log-init -g serial $USERNAME && \
    usermod -aG sudo $USERNAME
RUN echo "$USERNAME:$USERNAME" | chpasswd
RUN chsh -s /bin/bash $USERNAME
#RUN apt-get install -y net-tools lxde
#RUN apt-get install -y openbox
#RUN apt-get install -y lightdm

# Add files
ENV DISPLAY :1
WORKDIR $HOME
RUN mkdir /root/.vnc
RUN mkdir ./ros-playground
RUN mkdir ./gazebo
RUN mkdir -p ./.config/nautilus
RUN mkdir -p /var/log/supervisor
RUN rm .bashrc
RUN curl https://raw.githubusercontent.com/git/git/master/contrib/completion/git-prompt.sh > .git-prompt.bash
COPY .bashrc .
COPY .tmux.conf .
COPY .vimrc .
COPY autumn.jpg .
#ADD image /
#COPY xstartup /root/.vnc/xstartup
#COPY passwd /root/.vnc/passwd
#COPY x11vnc.pass /root/x11vnc.pass
COPY supervisord.conf /etc/supervisor/conf.d/supervisord.conf
COPY .gazebo ./.gazebo

RUN chown -R serial:serial .bashrc \
                           .tmux.conf \
                           .vimrc \
                           ros-playground \
                           ros-playground/.*
#RUN chmod 600 /root/.vnc/passwd
#RUN chmod 400 /root/x11vnc.pass

# TEMPORARY - SHOULD BE HANDLED BY ROSDEP
RUN apt-get install -y\
        ros-kinetic-ros-control \
        ros-kinetic-ros-controllers \
        ros-kinetic-gazebo-ros-control

# Install ROS dependencies
WORKDIR $HOME/ros-playground
USER serial
RUN echo "export WS=~/" >> $HOME/.bashrc
RUN rosdep update
RUN ls -lFa
#RUN /bin/bash --login -c "catkin_make && catkin_make install"

# Setup VNC
ENV USER root
EXPOSE 5900

# Set XDRP to use TightVNC port
USER root
CMD    ["/usr/bin/supervisord"]
#CMD /usr/bin/vncserver :1 -geometry 1280x800 -depth 24 && tail -f /root/.vnc/*:1.log

