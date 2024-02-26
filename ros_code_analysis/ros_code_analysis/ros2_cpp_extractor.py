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

class Ros2CppExtractor():
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
    for call in (CodeQuery(gs).all_calls.get()):
        if "Publisher" in str(call):
            if len(call.arguments) > 1:
                name = analysis._extract_topic(call, topic_pos=0)
                msg_type = analysis._extract_message_type(call)
                queue_size = analysis._extract_queue_size(call, queue_pos=1)
                if name!="?" or msg_type!="?":
                    pub = RosModelMetamodel.Publisher(name=name,type=msg_type.replace(".msg",""))
                    publishers.append(pub)
        if "Subscription" in str(call):  # Subscription or Subscriber?
            if len(call.arguments) > 1:
                name = analysis._extract_topic(call, topic_pos=0)
                msg_type = analysis._extract_message_type(call)
                queue_size = analysis._extract_queue_size(call, queue_pos=1)
                if name!="?" or msg_type!="?":
                    sub = RosModelMetamodel.Subscriber(name=name,type=msg_type.replace(".msg",""))
                    subscribers.append(sub)
        if "Service" in str(call) and "::srv::" in str(call): #or?
            if len(call.arguments) > 1:
                name = analysis._extract_topic(call, topic_pos=0)
                srv_type = analysis._extract_message_type(call)
                queue_size = analysis._extract_queue_size(call, queue_pos=1)
                if name!="?" or srv_type!="?":
                    ss = RosModelMetamodel.ServiceServer(name=name,type=srv_type.replace(".srv",""))
                    serviceservers.append(ss)
        if "Client" in str(call) and "::srv::" in str(call):
            if len(call.arguments) > 1:
                name = analysis._extract_topic(call, topic_pos=0)
                srv_type = analysis._extract_message_type(call)
                queue_size = analysis._extract_queue_size(call, queue_pos=1)
                if name!="?" or srv_type!="?":
                    sc = RosModelMetamodel.ServiceClient(name=name,type=srv_type.replace(".srv",""))
                    serviceclients.append(sc)
        if "Server" in str(call) and "::action::" in str(call):
            if len(call.arguments) > 1:
                name = analysis._extract_topic(call, topic_pos=0)
                act_type = analysis._extract_message_type(call)
                queue_size = analysis._extract_queue_size(call, queue_pos=1)
                if name!="?" or act_type!="?":
                    acs = RosModelMetamodel.ActionServer(name=name,type=act_type.replace(".action",""))
                    actionservers.append(acs)
        if "Client" in str(call) and "::action::" in str(call):
            if len(call.arguments) > 1:
                name = analysis._extract_topic(call, topic_pos=0)
                act_type = analysis._extract_message_type(call)
                queue_size = analysis._extract_queue_size(call, queue_pos=1)
                if name!="?" or act_type!="?":
                    ac = RosModelMetamodel.ActionServer(name=name,type=act_type.replace(".action",""))
                    actionclients.append(ac)
        if "declare_parameter" in str(call):
            if len(call.arguments) > 1:
                name = analysis._extract_topic(call, topic_pos=0)
                default_value = resolve_expression(call.arguments[1])
                param_type = RosCommonExtractor.transform_type(resolve_expression(call))
                if not default_value:
                    param = RosModelMetamodel.Parameter(name=name,type=param_type)
                else:
                    param = RosModelMetamodel.Parameter(name=name,type=param_type,value=str(default_value))
                parameters.append(param)

    return publishers, subscribers, serviceservers, serviceclients, actionservers, actionclients, parameters