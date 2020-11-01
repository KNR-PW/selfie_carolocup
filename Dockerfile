FROM ros:noetic-robot

################################################################################
# Install prerequisites
################################################################################

RUN apt-get update

#wget
RUN apt-get install -y wget

# basler driver
RUN wget "https://www.baslerweb.com/fp-1551786516/media/downloads/software/pylon_software/pylon-5.2.0.13457-x86_64.tar.gz" 2>&1
RUN tar xvzf pylon-5.2.0.13457-x86_64.tar.gz >/dev/null
RUN tar -C /opt -xzf pylon-5.2.0.13457-x86_64/pylonSDK*.tar.gz >/dev/null
RUN rm -r pylon-5.2.0.13457-x86_64
RUN rm pylon-5.2.0.13457-x86_64.tar.gz
RUN sh -c 'echo "yaml https://raw.githubusercontent.com/magazino/pylon_camera/indigo-devel/rosdep/pylon_sdk.yaml " > /etc/ros/rosdep/sources.list.d/15-plyon_camera.list' 
RUN rosdep update 2>/dev/null

# python
RUN apt-get install -y python3-catkin-lint >/dev/null 2>&1

################################################################################
# Install selfie dependencies
################################################################################

COPY ./src /tmp/selfie/src
WORKDIR /tmp/selfie
RUN rosdep install -y --from-paths src --ignore-src
WORKDIR /root
RUN rm -r /tmp/selfie
