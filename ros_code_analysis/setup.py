#!/usr/bin/env python
#
# Copyright 2024 Fraunhofer Institute for Manufacturing Engineering and Automation (IPA)
#
# Nadia Hammoudeh Garcia
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#	http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from setuptools import setup

setup(
    name='ros_code_analysis',
    version='0.0.1',    
    description='Static code analysis plugin based on HAROS to extract information from ROS nodes',
    url='https://github.com/ipa320/roscode2model',
    author='Nadia Hammoudeh Garcia',
    author_email='nadia.hammoudeh.garcia@ipa.fraunhofer.de',
    license='Apache 2.0',
    packages=['ros_code_analysis'],
    install_requires=['haros',
                      'bonsai',                     
                      ],

)

