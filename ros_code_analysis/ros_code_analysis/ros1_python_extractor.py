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

from bonsai.analysis import CodeQuery, resolve_expression
from ros_common_extractor import RosCommonExtractor
import ros2model.core.metamodels.metamodel_ros as RosModelMetamodel

class Ros1PythonExtractor():
  def extract_primitives(node, parser, analysis):
    gs = parser.global_scope
    node.source_tree = parser.global_scope
    publishers=[]
    subscribers=[]
    serviceservers=[]
    serviceclients=[]
    actionservers=[]
    actionclients=[]
    parameters=[]

    msgs_list = []
    pkgs_list = []
    for imp_name in parser.imported_names_list:
        s = str(imp_name)
        if "msg" in s or "srv" in s:
            ss = s.split(".")
            if len(ss) < 2:
                continue
            if ss[-1] == "msg" or ss[-1] == "srv":
                pkgs_list.append(ss[0])
            elif ss[1] == "msg" or ss[1] == "srv":
                msgs_list.append((ss[0], ss[2]))
            else:
                log.debug(("Python import with 'msg' or 'srv', "
                                "but unable to process it: ")
                                + s)
    for call in CodeQuery(gs).all_calls.get():
        if "rospy.Publisher" in str(call):
            if len(call.arguments) > 1:
                name = analysis._extract_topic(call, topic_pos=0)
                msg_type = analysis._extract_message_type(call, '', msgs_list, pkgs_list)
                pub = RosModelMetamodel.Publisher(name=name,type=msg_type.replace(".msg",""))
                publishers.append(pub)
        if "rospy.Subscriber" in str(call):
            if len(call.arguments) > 1:
                name = analysis._extract_topic(call, topic_pos=0)
                msg_type = analysis._extract_message_type(call, '', msgs_list, pkgs_list)
                sub = RosModelMetamodel.Subscriber(name=name,type=msg_type.replace(".msg",""))
                subscribers.append(sub)
        if "rospy.Service" in str(call):
            if len(call.arguments) > 1:
                name = analysis._extract_topic(call, topic_pos=0)
                srv_type = analysis._extract_message_type(call, '', msgs_list)
                ss = RosModelMetamodel.ServiceServer(name=name,type=srv_type.replace(".srv","").replace("Request",""))
                serviceservers.append(ss)
        if "rospy.ServiceProxy" in str(call):
            if len(call.arguments) > 1:
                name = analysis._extract_topic(call, topic_pos=0)
                srv_type = analysis._extract_message_type(call, '', msgs_list)
                sc = RosModelMetamodel.ServiceClient(name=name,type=srv_type.replace(".srv","").replace("Response",""))
                serviceclients.append(sc)
        if "rospy.set_param" in str(call):
            param_name = analysis._extract_topic(call, topic_pos=0)
            param_type = default_value = None
            default_value = resolve_expression(call.arguments[1])
            param = RosModelMetamodel.Parameter(name=param_name,type=param_type,value=default_value)
            parameters.append(param)