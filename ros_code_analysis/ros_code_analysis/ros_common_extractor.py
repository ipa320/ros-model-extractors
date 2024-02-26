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

class RosCommonExtractor():
  def transform_type(param_type):
    #if os.environ.get("ROS_VERSION") == "2":
    param_type=str(param_type)
    param_type=param_type[param_type.find("[")+1:param_type.find("]")]
    if param_type == 'double':
        return 'Double'
    elif param_type == 'bool':
        return 'Boolean'
    elif param_type == 'int' or param_type == 'long':
        return 'Integer'
    elif (param_type == 'str' or 'basic_string<char>' in param_type or param_type == 'std::string'):
        return 'String'
    #elif param_type == 'yaml':
        #return 'Struct'
    #elif 'std::vector' in param_type:
        #return 'List'
    else:
      return None