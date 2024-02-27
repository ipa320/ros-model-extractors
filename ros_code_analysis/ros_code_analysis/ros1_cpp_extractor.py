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

class Ros1CppExtractor():
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
            for call in (CodeQuery(gs).all_calls.where_name("SimpleActionServer").get()):
                if len(call.arguments) > 0:
                    name = analysis._extract_action(call)
                    action_type = analysis._extract_action_type(call).split("_<",1)[0]
                    if name!="?" or act_type!="?":
                        acs = RosModelMetamodel.ActionServer(name=name,type=act_type.replace(".action",""))
                        actionservers.append(acs)
            for call in (CodeQuery(gs).all_calls.where_name("SimpleActionClient").get()):
                if len(call.arguments) > 0:
                    name = analysis._extract_action(call)
                    action_type = analysis._extract_action_type(call).split("_<",1)[0]
                    if name!="?" or act_type!="?":
                        ac = RosModelMetamodel.ActionServer(name=name,type=act_type.replace(".action",""))
                        actionclients.append(ac)
            for call in (CodeQuery(gs).all_calls.where_name("advertise").where_result("ros::Publisher").get()):
                if len(call.arguments) > 1:
                    name = analysis._extract_topic(call, topic_pos=0)
                    msg_type = analysis._extract_message_type(call)
                    queue_size = analysis._extract_queue_size(call, queue_pos=1)
                    if name!="?" or msg_type!="?":
                        pub = RosModelMetamodel.Publisher(name=name,type=msg_type.replace(".msg",""))
                        publishers.append(pub)
            for call in (CodeQuery(gs).all_calls.where_name("subscribe").where_result("ros::Subscriber").get()):
                if len(call.arguments) > 1:
                    name = analysis._extract_topic(call, topic_pos=0)
                    msg_type = analysis._extract_message_type(call)
                    queue_size = analysis._extract_queue_size(call, queue_pos=1)
                    if name!="?" or msg_type!="?":
                        sub = RosModelMetamodel.Subscriber(name=name,type=msg_type.replace(".msg",""))
                        subscribers.append(sub)
            for call in (CodeQuery(gs).all_calls.where_name("advertiseService").where_result("ros::ServiceServer").get()):
                if len(call.arguments) > 1:
                    name = analysis._extract_topic(call)
                    srv_type = analysis._extract_message_type(call)
                    if name!="?" or srv_type!="?":
                        ss = RosModelMetamodel.ServiceServer(name=name,type=srv_type.replace(".srv","").replace("Request",""))
                        serviceservers.append(ss)
            for call in (CodeQuery(gs).all_calls.where_name("serviceClient").where_result("ros::ServiceClient").get()):
                if len(call.arguments) > 1:
                    name = analysis._extract_topic(call)
                    srv_type = analysis._extract_message_type(call)
                    if name!="?" or srv_type!="?":
                        sc = RosModelMetamodel.ServiceClient(name=name,type=srv_type.replace(".srv","").replace("Response",""))
                        serviceclients.append(sc)
            
            #PARAMETERS nhg:this needs review
            nh_prefix = "c:@N@ros@S@NodeHandle@"
            gets = ("getParam", "getParamCached", "param")
            reads = gets + ("hasParam", "searchParam")
            sets = ("setParam",)
            writes = sets + ("deleteParam",)
            for call in CodeQuery(gs).all_calls.where_name(reads).get():
                if (call.full_name.startswith("ros::NodeHandle") or (isinstance(call.reference, str) and call.reference.startswith(nh_prefix))):
                    param_type = default_value = None
                    param_name = analysis._extract_topic(call)
                    if call.name in gets:
                        param_type = RosCommonExtractor.transform_type(analysis._extract_param_type(call.arguments[1]))
                    if call.name == "param":
                        if len(call.arguments) > 2:
                            default_value = analysis._extract_param_value( call, arg_pos=2)
                        elif len(call.arguments) == 2:
                            default_value = analysis._extract_param_value( call, arg_pos=1)
                    if not ((default_value is None or default_value == "") and param_type is None):
                        param = RosModelMetamodel.Parameter(name=param_name,type=param_type,value=default_value)
                        parameters.append(param)
      
            for call in CodeQuery(gs).all_calls.where_name(writes).get():
                if (call.full_name.startswith("ros::NodeHandle") or (isinstance(call.reference, str) and call.reference.startswith(nh_prefix))):
                    param_type = value = None
                    param_name = analysis._extract_topic(call)
                    if len(call.arguments) >= 2 and call.name in sets:
                        param_type = RosCommonExtractor.transform_type(analysis._extract_param_type(call.arguments[1]))
                        value = analysis._extract_param_value(call, arg_pos=1)
                    if not ((default_value is None or default_value == "") and param_type is None):
                        param = RosModelMetamodel.Parameter(name=param_name,type=param_type,value=default_value)
                        parameters.append(param)
            ros_prefix = "c:@N@ros@N@param@"
            gets = ("get", "getCached", "param")
            reads = gets + ("has",)
            sets = ("set",)
            writes = sets + ("del",)
            for call in CodeQuery(gs).all_calls.where_name(reads).get():
                if (call.full_name.startswith("ros::param") or (isinstance(call.reference, str) and call.reference.startswith(ros_prefix))):
                  param_type = default_value = None
                  param_name = analysis._extract_topic(call)
                  if call.name == "param":
                      if call.name in gets:
                          param_type = RosCommonExtractor.transform_type(analysis._extract_param_type(call.arguments[1]))
                      if len(call.arguments) > 2:
                          default_value = analysis._extract_param_value(call, arg_pos=2)
                      elif len(call.arguments) == 2:
                          default_value = analysis._extract_param_value(call, arg_pos=1)
                      if not ((default_value is None or default_value == "") and param_type is None):
                        param = RosModelMetamodel.Parameter(name=param_name,type=param_type,value=default_value)
                        parameters.append(param)
            for call in CodeQuery(gs).all_calls.where_name(writes).get():
                if (call.full_name.startswith("ros::param") or (isinstance(call.reference, str) and call.reference.startswith(ros_prefix))):
                    param_type = value = None
                    if len(call.arguments) >= 2 and call.name in sets:
                        param_type = RosCommonExtractor.transform_type(analysis._extract_param_type(call.arguments[1]))
                        value = analysis._extract_param_value(call, arg_pos=1)
                    param_name = analysis._extract_topic(call)
                    if not ((default_value is None or default_value == "") and param_type is None):
                        param = RosModelMetamodel.Parameter(name=param_name,type=param_type,value=default_value)
                        parameters.append(param)
    return publishers, subscribers, serviceservers, serviceclients, actionservers, actionclients, parameters