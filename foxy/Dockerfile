# Use an official Python runtime as a parent image
FROM ros:foxy
ARG enable_ssh
ARG path_to_scripts
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
    libmagic-dev \
    python-is-python3

RUN if [ $enable_ssh ] ; then apt-get install -y openssh-server ; fi

ENV LD_LIBRARY_PATH $LD_LIBRARY_PATH:/usr/lib/llvm-10/lib

RUN pip3 install --upgrade pip
RUN pip3 install -Iv clang==10.0.1
RUN pip3 install -e git+https://github.com/timtadh/pyflwor.git#egg=pyflwor
RUN pip3 install -e git+https://github.com/ipa320/ros_model_parser.git#egg=ros_model_parser
RUN pip3 install -e git+https://github.com/git-afsantos/haros#egg=haros
RUN pip3 install -e git+https://github.com/ipa-nhg/bonsai@Fix#egg=bonsai-code

RUN apt-get update && apt-get install -y ros-foxy-desktop && apt upgrade -y

SHELL ["bash", "-c"]

RUN useradd -rm -d /home/extractor/  -u 1001 extractor
#RUN useradd -rm -d /home/extractor/ -s /bin/bash -g extractor -G sudo root -u 1001 extractor
RUN usermod -a -G sudo extractor
RUN usermod -a -G root extractor
RUN echo "extractor ALL=(ALL:ALL) NOPASSWD: ALL" | sudo tee /etc/sudoers.d/extractor
USER extractor
RUN mkdir -p /home/extractor/ws/src

RUN mkdir -p /home/extractor/results
RUN chown extractor:extractor /home/extractor/results
USER extractor
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
USER extractor

ENV CMAKE_CXX_COMPILER /usr/lib/llvm-10/bin/clang++
RUN source /opt/ros/$ROS_DISTRO/setup.bash;\
 cd /home/extractor/ws;\
 colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON ;\
 source /home/extractor/ws/install/setup.bash; \
 haros init

ENV PYTHON_VERSION 3
RUN echo 'source /home/extractor/ws/install/setup.bash' >> /home/extractor/.bashrc 

COPY ${path_to_scripts}messages_generator_runner.sh /
COPY ${path_to_scripts}generate_messages_model_helper.sh /
COPY ${path_to_scripts}haros_runner.sh /
COPY ${path_to_scripts}ros_model_extractor.py /
COPY ${path_to_scripts}test.sh /

EXPOSE 4005
#CMD sudo chown -R extractor:extractor /home/extractor/results
