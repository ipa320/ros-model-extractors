#!/usr/bin/env python
#
# Copyright 2019 Fraunhofer Institute for Manufacturing Engineering and Automation (IPA)
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

from Ros_Extractor import RosExtractor


class RosInterface:
  def __init__(self, name, ref):
    self.name = name
    self.ref = ref

class ros_component:
  def __init__(self, name, ns):
    self.name = ns+name if ns else name
    self.ns = ns
    self.pubs = []
    self.subs = []
    self.srvsrvs = []
    self.srvcls = []
    self.actsrvs = []
    self.actcls = []
  def add_interface(self, name, interface_type, ref):
    if interface_type == "pubs":
        self.pubs.append(RosInterface(name,ref))
    if interface_type == "subs":
        self.subs.append(RosInterface(name,ref))
    if interface_type == "srvsrvs":
        self.srvsrvs.append(RosInterface(name,ref))
    if interface_type == "srvcls":
        self.srvcls.append(RosInterface(name,ref))
    if interface_type == "actsrvs":
        self.actsrvs.append(RosInterface(name,ref))
    if interface_type == "actcls":
        self.actcls.append(RosInterface(name,ref))

def main(argv = None):
    extractor = RosExtractor()
    if extractor.launch():
        return 0
    return 1

if __name__== "__main__":
  main()
