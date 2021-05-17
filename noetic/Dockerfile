# Use an official Python runtime as a parent image
FROM ros:noetic

RUN echo 'debconf debconf/frontend select Noninteractive' | debconf-set-selections

RUN apt-get update && apt upgrade -y && apt-get install -y \
    cppcheck \
    cccc \
    clang-10 \
    git \
    libclang-10-dev \
    python3-pip \
    llvm-10-dev \
    wget \
    netbase \
    libmagic-dev

ENV LD_LIBRARY_PATH $LD_LIBRARY_PATH:/usr/lib/llvm-10/lib

RUN pip3 install --upgrade pip
RUN pip3 install -Iv clang==10.0.1
RUN pip3 install -e git+https://github.com/timtadh/pyflwor.git#egg=pyflwor
RUN pip3 install -e git+https://github.com/ipa320/haros.git@ros2#egg=haros
RUN pip3 install -e git+https://github.com/ipa320/ros_model_parser.git#egg=ros_model_parser
RUN pip3 install bonsai-code

RUN apt-get update && apt-get install -y ros-noetic-desktop && apt upgrade -y


SHELL ["bash", "-c"]

RUN mkdir -p /root/ws/src

# Set the working directory
WORKDIR .

ENV CMAKE_CXX_COMPILER /usr/lib/llvm-10/bin/clang++

RUN source /opt/ros/$ROS_DISTRO/setup.bash;\
 cd /root/ws/src;\
 catkin_init_workspace;\
 cd /root/ws;\
 catkin_make -DCMAKE_EXPORT_COMPILE_COMMANDS=1;\
 source /root/ws/devel/setup.bash; \
 haros init

RUN apt-get install -y python-is-python3
ENV PYTHON_VERSION 3

COPY haros_runner.sh /
COPY ros_model_extractor.py /

EXPOSE 4000

CMD []
