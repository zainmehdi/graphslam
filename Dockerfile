FROM ubuntu:trusty

MAINTAINER David Swords "david@swords.computer"

ENV DEBIAN_FRONTEND noninteractive

RUN apt-get update
RUN apt-get install -y build-essential git cmake g++ libboost-all-dev libtbb-dev 
RUN cd home && git clone http://github.com/davidswords/libgtsam.git
RUN cd home/libgtsam && mkdir build && cd build && cmake .. && make install

RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
RUN apt-get update
RUN apt-get install -y dialog apt-utils
RUN apt-get install -y ros-indigo-desktop
RUN rosdep init
RUN rosdep update
RUN echo "source /opt/ros/indigo/setup.bash" >> /home/.bashrc
RUN source /home/.bashrc

