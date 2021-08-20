# Use an official Python runtime as a parent image
FROM osrf/ros:melodic-desktop
ARG enable_ssh
ARG path_to_scripts

RUN echo 'debconf debconf/frontend select Noninteractive' | debconf-set-selections

RUN apt-get update && apt upgrade -y && apt-get install -y \
    cppcheck \
    cccc \
    clang-10 \
    git \
    libclang-10-dev \
    python-pip \
    llvm-10-dev \
    wget \
    netbase \
    libmagic-dev

RUN if [ $enable_ssh ] ; then apt-get install -y openssh-server ; fi

ENV LD_LIBRARY_PATH $LD_LIBRARY_PATH:/usr/lib/llvm-10/lib

RUN pip install --upgrade pip
RUN pip install -Iv clang==10.0.1
RUN pip install -e git+https://github.com/timtadh/pyflwor.git#egg=pyflwor
RUN pip install -e git+https://github.com/ipa320/ros_model_parser.git#egg=ros_model_parser
RUN pip install haros
RUN pip install bonsai-code

RUN apt-get update && apt-get install -y ros-melodic-desktop && apt upgrade -y

SHELL ["bash", "-c"]

RUN useradd -rm -d /home/extractor/ -s /bin/bash -g root -G sudo -u 1001 extractor
RUN adduser extractor sudo
RUN echo "extractor ALL=(ALL:ALL) NOPASSWD: ALL" | sudo tee /etc/sudoers.d/extractor
USER extractor
RUN mkdir -p /home/extractor/ws/src
WORKDIR /home/extractor

# SSH CONFIGURATION
RUN if [ $enable_ssh ] ; then mkdir -p /home/extractor/.ssh/ && \
    chmod 0700 /home/extractor/.ssh  && \
    touch /home/extractor/.ssh/authorized_keys && \
    chmod 600 /home/extractor/.ssh/authorized_keys && \
    touch /home/extractor/.ssh/config && \
    chmod 600 /home/extractor/.ssh/config ; fi

COPY --chown=extractor:root ssh_config/ /keys/
RUN if [ $enable_ssh ] ; then  cat /keys/ssh_key.pub >> /home/extractor/.ssh/authorized_keys ; fi
RUN if [ $enable_ssh ] ; then  cat /keys/config >> /home/extractor/.ssh/config ; fi
####

ENV CMAKE_CXX_COMPILER /usr/lib/llvm-10/bin/clang++
RUN source /opt/ros/$ROS_DISTRO/setup.bash;\
 cd /home/extractor/ws/src;\
 rosdep update;\
 catkin_init_workspace;\
 cd /home/extractor/ws;\
 catkin_make -DCMAKE_EXPORT_COMPILE_COMMANDS=1;\
 source /home/extractor/ws/devel/setup.bash; \
 haros init

ENV PYTHON_VERSION 2

COPY ${path_to_scripts}haros_runner.sh /
COPY ${path_to_scripts}ros_model_extractor.py /
COPY ${path_to_scripts}test.sh /

EXPOSE 4000

ENTRYPOINT  if [ $enable_ssh ] ; then  sudo service ssh start && bash ; fi

CMD []
