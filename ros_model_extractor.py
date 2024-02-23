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
from ros2model.api.model_generator.component_generator import ComponentGenerator
import ros2model.core.metamodels.metamodel_ros as RosModelMetamodel

import rospkg
from copy import deepcopy
#import ament_index_python 
from haros.extractor import NodeExtractor, RoscppExtractor, RospyExtractor
from haros.metamodel import Node, Package, RosName, SourceFile
from haros.launch_parser import LaunchParser, LaunchParserError, NodeTag
from haros.cmake_parser import RosCMakeParser
from bonsai.analysis import CodeQuery, resolve_expression

try:
    from bonsai.cpp.clang_parser import CppAstParser
except ImportError:
    CppAstParser = None
from bonsai.py.py_parser import PyAstParser

class RosExtractor():
  def launch(self):
    self.parse_arg()
    ws = self.args.worspace_path
    clang_version= self.args.clang_version
    
    #BONSAI PARSER
    parser = CppAstParser(workspace = ws)
    parser.set_library_path("/usr/lib/llvm-"+clang_version+"/lib")
    parser.set_standard_includes("/usr/lib/llvm-"+clang_version+"/lib/clang/"+clang_version+".0.0/include")
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
      self.pkg_type="CatkinPackage"
    elif os.environ.get("ROS_VERSION") == "2":
      self.pkg.path= self.args.path_to_src
      self.pkg_type="AmentPackage"
    roscomponent = None
    #HAROS NODE EXTRACTOR

    srcdir = self.pkg.path[len(ws):]
    srcdir = os.path.join(ws, srcdir.split(os.sep, 1)[0])
    bindir = os.path.join(ws, "build")
    #HAROS CMAKE PARSER
    parser = RosCMakeParser(srcdir, bindir, pkgs = [self.pkg])
    model_str = ""
    if os.path.isfile(os.path.join(self.pkg.path, "CMakeLists.txt")):
        parser.parse(os.path.join(self.pkg.path, "CMakeLists.txt"))
        node_name_dict = deepcopy(parser.executables)
        node_name_dict.update(deepcopy(parser.libraries))
        for target in node_name_dict.values():
            print("INFO: Found artifact: "+target.output_name)
            if (self.args.a):
                node_name = target.output_name
                node = Node(node_name, self.pkg, rosname=RosName(node_name))
                for file_in in target.files:
                  full_path = file_in
                  relative_path = full_path.replace(self.pkg.path+"/","").rpartition("/")[0]
                  file_name = full_path.rsplit('/', 1)[-1]
                  source_file = SourceFile(file_name, relative_path , self.pkg)
                  node.source_files.append(source_file)
            else:
              if target.output_name == node_name:
                  node = Node(node_name, self.pkg, rosname=RosName(node_name))
                  for file_in in target.files:
                      full_path = file_in
                      relative_path = full_path.replace(self.pkg.path+"/","").rpartition("/")[0]
                      file_name = full_path.rsplit('/', 1)[-1]
                      source_file = SourceFile(file_name, relative_path , self.pkg)
                      node.source_files.append(source_file)
              else:
                continue
            if node.language == "cpp":
              parser = CppAstParser(workspace = ws)
              analysis = RoscppExtractor(self.pkg, ws)
            if node.language == "python":
              parser = PyAstParser(workspace = ws)
              analysis = RospyExtractor(self.pkg, ws)
          #node.source_tree = parser.global_scope
            for sf in node.source_files:
              try:
                if parser.parse(sf.path) is not None:
                  # ROS MODEL EXTRACT PRIMITIVES 
                  if node.language == "python":
                      node_name=node_name.replace(".py","")
                  graph_name = RosModelMetamodel.GraphName(name=node_name, namespace="", full_name=node_name)
                  try:
                    publishers = self.extract_primitives(node, parser, analysis, roscomponent, pkg_name, node_name, node_name)
                    #, subscribers, serviceservers, serviceclients, actionservers, actionclients, parameters = self.extract_primitives(node, parser, analysis, roscomponent, pkg_name, node_name, node_name)
                    RosModel_node=RosModelMetamodel.Node(name=graph_name,
                            publisher=publishers)
                  except error:
                    print("Interfaces not found")
                    print(error)
                    RosModel_node=RosModelMetamodel.Node(name=graph_name)
                  RosModel_artifact=RosModelMetamodel.Artifact(name=node_name, node=[RosModel_node])
                  RosModel_package=RosModelMetamodel.Package(name=self.args.package_name, artifact=[RosModel_artifact])
                  node_generator = ComponentGenerator()
                  node_generator.generate_a_file(
                    model=RosModel_package,
                    output_dir=self.args.model_path,
                    filename=node_name+".ros2",
                  )
                else:
                  print("The model couldn't be extracted")
              except:
                  pass
            if rossystem is not None and roscomponent is not None:
                rossystem.add_component(roscomponent)
    if self.args.output:
        print(model_str)

  def transform_type(self, param_type):
    if os.environ.get("ROS_VERSION") == "2":
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


  def extract_primitives(self, node, parser, analysis, roscomponent, pkg_name, node_name, art_name):
    gs = parser.global_scope
    node.source_tree = parser.global_scope
    publishers=[]
    subscribers=[]
    erviceservers=[]
    serviceclients=[]
    actionservers=[]
    actionclients=[]
    parameters=[]
    if os.environ.get("ROS_VERSION") == "2":
        #ROS2
        if node.language == "cpp":
          for call in (CodeQuery(gs).all_calls.get()):
              if "Publisher" in str(call):
                if len(call.arguments) > 1:
                  name = analysis._extract_topic(call, topic_pos=0)
                  msg_type = analysis._extract_message_type(call)
                  queue_size = analysis._extract_queue_size(call, queue_pos=1)
                  if name!="?" or msg_type!="?":
                    pub = RosModelMetamodel.Publisher(name=name,type=msg_type.replace("/",".").replace(".msg",""))
                    publishers.append(pub)
    return publishers#, subscribers, serviceservers, serviceclients, actionservers, actionclients, parameters


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
      parser.add_argument('--name', required=False, dest='name')
      parser.add_argument('--ws', required=True, dest='worspace_path')
      parser.add_argument('--path-to-src', required=False, dest='path_to_src')
      parser.add_argument('--repo', required=False, dest='repo')
      parser.add_argument('-a', action='store_true')
      parser.add_argument('--clang-version', required=True, dest='clang_version')
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
