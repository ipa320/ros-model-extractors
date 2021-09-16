# Use an official Python runtime as a parent image
FROM osrf/ros:melodic-desktop

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

ENV LD_LIBRARY_PATH $LD_LIBRARY_PATH:/usr/lib/llvm-10/lib

RUN pip install --upgrade pip
RUN pip install -Iv clang==10.0.1
RUN pip install -e git+https://github.com/timtadh/pyflwor.git#egg=pyflwor
#RUN pip install -e git+https://github.com/ipa320/ros_model_parser.git#egg=ros_model_parser
RUN pip install -e git+https://github.com/ipa-nhg/ros_model_parser.git@GitRepo#egg=ros_model_parser
RUN pip install -e git+https://github.com/git-afsantos/haros#egg=haros
RUN pip install -e git+https://github.com/ipa-nhg/bonsai@Fix#egg=bonsai-code
#RUN pip install haros
#RUN pip install bonsai-code

RUN apt-get update && apt-get install -y ros-melodic-desktop && apt upgrade -y

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

ENV PYTHON_VERSION 2

RUN ls
COPY haros_runner.sh /
COPY ros_model_extractor.py /
COPY test.sh /

EXPOSE 4000

CMD []
