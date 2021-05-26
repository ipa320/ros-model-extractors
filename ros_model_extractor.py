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

import os
import argparse
import subprocess
from ros_model_generator.rosmodel_generator import RosModelGenerator
import ros_metamodels.ros_metamodel_core as RosModelMetamodel

import rospkg
#import ament_index_python 
from haros.extractor import NodeExtractor, RoscppExtractor, RospyExtractor
from haros.metamodel import Node, Package, RosName, SourceFile
from haros.launch_parser import LaunchParser, LaunchParserError, NodeTag
from haros.cmake_parser import RosCMakeParser
from bonsai.analysis import CodeQuery

try:
    from bonsai.cpp.clang_parser import CppAstParser
except ImportError:
    CppAstParser = None
from bonsai.py.py_parser import PyAstParser

class RosExtractor():
  def launch(self):
    self.parse_arg()
    ws = self.args.worspace_path

    #BONSAI PARSER
    parser = CppAstParser(workspace = ws)
    parser.set_library_path("/usr/lib/llvm-10/lib")
    parser.set_standard_includes("/usr/lib/llvm-10/lib/clang/10.0.0/include")
    db_dir = os.path.join(ws, "build")
    if os.path.isfile(os.path.join(db_dir, "compile_commands.json")):
        parser.set_database(db_dir)
    else:
      print("The compile_commands.json file can't be found")
    if (self.args.node):
        self.extract_node(self.args.name, self.args.name, self.args.package_name, None, ws, None)
  def extract_node(self, name, node_name, pkg_name, ns, ws, rossystem):
    self.pkg = Package(pkg_name)
    if os.environ.get("ROS_VERSION") == "1":
      rospack = rospkg.RosPack()
      self.pkg.path = rospack.get_path(pkg_name)
    elif os.environ.get("ROS_VERSION") == "2":
      self.pkg.path= self.args.path_to_src
    roscomponent = None
    #HAROS NODE EXTRACTOR
    node = Node(node_name, self.pkg, rosname=RosName(node_name))
    srcdir = self.pkg.path[len(ws):]
    srcdir = os.path.join(ws, srcdir.split(os.sep, 1)[0])
    bindir = os.path.join(ws, "build")
    #HAROS CMAKE PARSER
    parser = RosCMakeParser(srcdir, bindir, pkgs = [self.pkg])
    model_str = ""
    if os.path.isfile(os.path.join(self.pkg.path, "CMakeLists.txt")):
        parser.parse(os.path.join(self.pkg.path, "CMakeLists.txt"))
        for target in parser.executables.values():
            print("INFO: Found artifact: "+target.output_name)
            if target.output_name == node_name:
                for file_in in target.files:
                    full_path = file_in
                    relative_path = full_path.replace(self.pkg.path+"/","").rpartition("/")[0]
                    file_name = full_path.rsplit('/', 1)[-1]
                    source_file = SourceFile(file_name, relative_path , self.pkg)
                    node.source_files.append(source_file)
        if node.language == "cpp":
            parser = CppAstParser(workspace = ws)
            analysis = RoscppExtractor(self.pkg, ws)
        if node.language == "py":
            parser = PyAstParser(workspace = ws)
            analysis = RospyExtractor(self.pkg, ws)
        #node.source_tree = parser.global_scope
        for sf in node.source_files:
            try:
                if parser.parse(sf.path) is not None:
                    # ROS MODEL EXTRACT PRIMITIVES 
                    if node.language == "py":
                        node_name=node_name.replace(".py","")
                    RosModel_node=RosModelMetamodel.Node(node_name)
                    try:
                        self.extract_primitives(node, parser, analysis, RosModel_node, roscomponent, pkg_name, node_name, name)
                        # SAVE ROS MODEL
                        ros_model = RosModelGenerator()
                        ros_model.create_model_from_node(self.pkg.name,name, RosModel_node)
                        model_str = ros_model.generate_ros_model(self.args.model_path+"/"+name+".ros")
                    except error:
                        print("The interfaces can't be extracted "+error)
                else:
                  print("The model couldn't be extracted")
            except:
                pass
        if rossystem is not None and roscomponent is not None:
            rossystem.add_component(roscomponent)
    if self.args.output:
        print(model_str)

  def extract_primitives(self, node, parser, analysis, RosModel_node, roscomponent, pkg_name, node_name, art_name):
    gs = parser.global_scope
    node.source_tree = parser.global_scope
    if os.environ.get("ROS_VERSION") == "1":
        if node.language == "cpp":
            #print(CodeQuery(gs).all_calls.get())
            for call in (CodeQuery(gs).all_calls.where_name("SimpleActionServer").get()):
                if len(call.arguments) > 0:
                  name = analysis._extract_action(call)
                  action_type = analysis._extract_action_type(call).split("_<",1)[0]
                  RosModel_node.add_action_server(name,action_type.replace("/","."))
                  #roscomponent.add_interface(name,"actsrvs", pkg_name+"."+art_name+"."+node_name+"."+name)
            for call in (CodeQuery(gs).all_calls.where_name("SimpleActionClient").get()):
                if len(call.arguments) > 0:
                  name = analysis._extract_action(call)
                  action_type = analysis._extract_action_type(call).split("_<",1)[0]
                  RosModel_node.add_action_client.append(name,action_type.replace("/","."))
                  #roscomponent.add_interface(name,"actcls", str(pkg_name)+"."+str(art_name)+"."+str(node_name)+"."+str(name))
            for call in (CodeQuery(gs).all_calls.where_name("advertise").where_result("ros::Publisher").get()):
                if len(call.arguments) > 1:
                  name = analysis._extract_topic(call, topic_pos=0)
                  msg_type = analysis._extract_message_type(call)
                  queue_size = analysis._extract_queue_size(call, queue_pos=1)
                  RosModel_node.add_publisher(name, msg_type.replace("/","."))
                  #roscomponent.add_interface(name,"pubs", pkg_name+"."+art_name+"."+node_name+"."+name)
            for call in (CodeQuery(gs).all_calls.where_name("subscribe").where_result("ros::Subscriber").get()):
                if len(call.arguments) > 1:
                  name = analysis._extract_topic(call, topic_pos=0)
                  msg_type = analysis._extract_message_type(call)
                  queue_size = analysis._extract_queue_size(call, queue_pos=1)
                  RosModel_node.add_subscriber(name, msg_type.replace("/","."))
                  #roscomponent.add_interface(name,"subs", pkg_name+"."+art_name+"."+node_name+"."+name)
            for call in (CodeQuery(gs).all_calls.where_name("advertiseService").where_result("ros::ServiceServer").get()):
                if len(call.arguments) > 1:
                  name = analysis._extract_topic(call)
                  srv_type = analysis._extract_message_type(call)
                  RosModel_node.add_service_server(name, srv_type.replace("/",".").replace("Request",""))
                  #roscomponent.add_interface(name,"srvsrvs", pkg_name+"."+art_name+"."+node_name+"."+name)
            for call in (CodeQuery(gs).all_calls.where_name("serviceClient").where_result("ros::ServiceClient").get()):
                if len(call.arguments) > 1:
                  name = analysis._extract_topic(call)
                  srv_type = analysis._extract_message_type(call)
                  RosModel_node.add_service_client(name, srv_type.replace("/",".").replace("Response",""))
                  #roscomponent.add_interface(name,"srvcls", pkg_name+"."+art_name+"."+node_name+"."+name)

        if node.language == "py":
            msgs_list=[]
            for i in parser.imported_names_list:
                if "msg" in str(i) or "srv" in str(i):
                    msgs_list.append((i.split(".")[0],i.split(".")[2]))
            for call in (CodeQuery(gs).all_calls.where_name(('Publisher','rospy.Publisher'))).get():
                if len(call.arguments) > 1:
                  ns, name = analysis._extract_topic(call)
                  msg_type = analysis._extract_message_type(call, 'data_class', msgs_list)
                  queue_size = analysis._extract_queue_size(call )
                  RosModel_node.add_publisher(name, msg_type.replace("/","."))
                  roscomponent.add_interface(name,"pubs", pkg_name+"."+art_name+"."+node_name+"."+name)
            for call in (CodeQuery(gs).all_calls.where_name(('Subscriber', 'rospy.Subscriber'))).get():
                if len(call.arguments) > 1:
                  ns, name = analysis._extract_topic(call)
                  msg_type = analysis._extract_message_type(call, 'data_class', msgs_list)
                  queue_size = analysis._extract_queue_size(call )
                  RosModel_node.add_subscriber(name, msg_type.replace("/","."))
                  roscomponent.add_interface(name,"subs", pkg_name+"."+art_name+"."+node_name+"."+name)
            for call in (CodeQuery(gs).all_calls.where_name(analysis.all_rospy_names('service-def'))).get():
                if len(call.arguments) > 1:
                  ns, name = analysis._extract_topic(call)
                  srv_type = analysis._extract_message_type(call, 'service_class', msgs_list)
                  RosModel_node.add_service_server(name, srv_type.replace("/",".").replace("Request",""))
                  roscomponent.add_interface(name,"srvsrvs", pkg_name+"."+art_name+"."+node_name+"."+name)
            for call in (CodeQuery(gs).all_calls.where_name(analysis.all_rospy_names('service-call'))).get():
                if len(call.arguments) > 1:
                  ns, name = analysis._extract_topic(call)
                  srv_type = analysis._extract_message_type(call, 'service_class', msgs_list)
                  RosModel_node.add_service_client(name, srv_type.replace("/",".").replace("Response",""))
                  roscomponent.add_interface(name,"srvcls", pkg_name+"."+art_name+"."+node_name+"."+name)
    if os.environ.get("ROS_VERSION") == "2":
        #ROS2
        if node.language == "cpp":
          for call in (CodeQuery(gs).all_calls.get()):
              if "Publisher" in str(call):
                #print(call)
                if len(call.arguments) > 1:
                  name = analysis._extract_topic(call, topic_pos=0)
                  msg_type = analysis._extract_message_type(call)
                  queue_size = analysis._extract_queue_size(call, queue_pos=1)
                  if name!="?" or msg_type!="?":
                    RosModel_node.add_publisher(name, msg_type.replace("/",".").replace(".msg",""))
          for call in (CodeQuery(gs).all_calls.get()):
              if "Subscription" in str(call):
                #print(call)
                if len(call.arguments) > 1:
                  name = analysis._extract_topic(call, topic_pos=0)
                  msg_type = analysis._extract_message_type(call)
                  queue_size = analysis._extract_queue_size(call, queue_pos=1)
                  if name!="?" or msg_type!="?":
                    RosModel_node.add_subscriber(name, msg_type.replace("/",".").replace(".msg",""))
          for call in (CodeQuery(gs).all_calls.get()):
              if "Service" in str(call):
                #print(call)
                if len(call.arguments) > 1:
                  name = analysis._extract_topic(call, topic_pos=0)
                  srv_type = analysis._extract_message_type(call)
                  queue_size = analysis._extract_queue_size(call, queue_pos=1)
                  if name!="?" or srv_type!="?":
                    RosModel_node.add_service_server(name, srv_type.replace("/",".").replace(".srv",""))
          for call in (CodeQuery(gs).all_calls.get()):
              if "Client" in str(call):
                #print(call)
                if len(call.arguments) > 1:
                  name = analysis._extract_topic(call, topic_pos=0)
                  srv_type = analysis._extract_message_type(call)
                  queue_size = analysis._extract_queue_size(call, queue_pos=1)
                  if name!="?" or srv_type!="?":
                    RosModel_node.add_service_client(name, srv_type.replace("/",".").replace(".srv",""))

  def parse_arg(self):
    parser = argparse.ArgumentParser()
    mutually_exclusive = parser.add_mutually_exclusive_group()
    mutually_exclusive.add_argument('--node', '-n', help="node analyse", action='store_true')
    mutually_exclusive.add_argument('--launch', '-l', help="launch analyse", action='store_true')
    parser.add_argument('--model-path', help='path to the folder in which the model files should be saved',
                        default='./',
                        nargs='?', const='./')
    parser.add_argument('--output', help='print the model output')
    parser.add_argument('--package', required=True, dest='package_name')
    parser.add_argument('--name', required=True, dest='name')
    parser.add_argument('--ws', required=True, dest='worspace_path')
    parser.add_argument('--path-to-src', required=False, dest='path_to_src')
    self.args = parser.parse_args()


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
