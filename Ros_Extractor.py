import os
import argparse
import subprocess
import rospkg
#import ament_index_python
from ros_model_generator.rosmodel_generator import RosModelGenerator
import ros_metamodels.ros_metamodel_core as RosModelMetamodel 
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

    # BONSAI PARSER
    parser = CppAstParser(workspace = ws)
    parser.set_library_path("/usr/lib/llvm-14/lib")
    parser.set_standard_includes("/usr/lib/llvm-14/lib/clang/14.0.0/include")
    db_dir = os.path.join(ws, "build")
    if os.path.isfile(os.path.join(db_dir, "compile_commands.json")):
        parser.set_database(db_dir)
    else:
      print("The compile_commands.json file can't be found")
    if (self.args.node):
        self.extract_node(self.args.name, self.args.name, self.args.package_name, None, ws, None)
  
  def extract_node(self, name, node_name, pkg_name, ns, ws, rossystem):
    self.pkg = Package(pkg_name)
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
        for target in parser.executables.values():
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
                      RosModel_node=RosModelMetamodel.Node(node_name)
                      try:
                          self.extract_primitives(node, parser, analysis, RosModel_node, roscomponent, pkg_name, node_name, node_name)
                          # SAVE ROS MODEL
                          ros_model = RosModelGenerator()
                          ros_model.create_model_from_node(self.pkg.name,node_name, RosModel_node, self.args.repo, self.pkg_type)
                          print("Save model in:")
                          print(self.args.model_path+"/"+node_name+".ros")
                          model_str = ros_model.generate_ros_model(self.args.model_path+"/"+node_name+".ros")
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
    #ROS2
    if node.language == "cpp":
        for call in (CodeQuery(gs).all_calls.get()):
            if "Publisher" in str(call):
                print("Printing calll========================", call)
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
        if "Service" in str(call) and "::srv::" in str(call):
        #print(call)
            if len(call.arguments) > 1:
                name = analysis._extract_topic(call, topic_pos=0)
                srv_type = analysis._extract_message_type(call)
                queue_size = analysis._extract_queue_size(call, queue_pos=1)
                print(name + " " + srv_type)
                if name!="?" or srv_type!="?":
                    RosModel_node.add_service_server(name, srv_type.replace("/",".").replace(".srv",""))
    for call in (CodeQuery(gs).all_calls.get()):
        if "Client" in str(call) and "::srv::" in str(call):
        #print(call)
            if len(call.arguments) > 1:
                name = analysis._extract_topic(call, topic_pos=0)
                srv_type = analysis._extract_message_type(call)
                queue_size = analysis._extract_queue_size(call, queue_pos=1)
                print(name + " " + srv_type)
                if name!="?" or srv_type!="?":
                    RosModel_node.add_service_client(name, srv_type.replace("/",".").replace(".srv",""))
        if "Client" in str(call) and "::action::" in str(call):
            if len(call.arguments) > 1:
                name = analysis._extract_topic(call, topic_pos=0)
                act_type = analysis._extract_message_type(call)
                queue_size = analysis._extract_queue_size(call, queue_pos=1)
                if name!="?" or act_type!="?":
                    RosModel_node.add_action_client(name, act_type.replace("/",".").replace(".action",""))
        if "Server" in str(call) and "::action::" in str(call):
            if len(call.arguments) > 1:
                name = analysis._extract_topic(call, topic_pos=0)
                act_type = analysis._extract_message_type(call)
                queue_size = analysis._extract_queue_size(call, queue_pos=1)
                if name!="?" or act_type!="?":
                    RosModel_node.add_action_server(name, act_type.replace("/",".").replace(".action",""))
    #PARAMETERS ROS2          
    params=[]
    written_params=[]
    for call in (CodeQuery(gs).all_calls.get()):
        param_prefix = "c:@N@rclcpp@S@Node@F@"
        declare_params = ("get_parameter","declare_parameter")
        if (call.full_name.startswith("rclcpp::Node") or (isinstance(call.reference, str)) and call.reference.startswith(param_prefix)):
            param_type = default_value = None
            if(call.name in declare_params) and len(call.arguments) > 1:
                param_name = analysis._extract_topic(call, topic_pos=0)
                if len(call.arguments) == 2:
                    param_type= self.transform_type(resolve_expression(call.arguments[1]))
                    params.append((param_name, param_type))
                elif len(call.arguments) > 2 and not ('[rclcpp::ParameterValue] (default)' in  str(resolve_expression(call.arguments[1]))):
                    default_value = resolve_expression(call.arguments[1])
                    param_type = self.transform_type(resolve_expression(call))
                    params.append((param_name, param_type, default_value))
    for parameter in params:
        param_name_ = param_type_ = default_value_ = None
        if len(parameter) > 2:
            param_name_, param_type_, default_value_ = parameter
            if not ((default_value_ is None or default_value_ == "") and param_type_ is None):
                RosModel_node.add_parameter(param_name_.replace(".","/"), default_value_ , param_type_, None)
            written_params.append(param_name_)
        elif len(parameter) == 2:
            param_name_, param_type_ = parameter
            if not (param_type_ is None) and not (param_name_ in written_params):
                RosModel_node.add_parameter(param_name_.replace(".","/"), default_value_ , param_type_, None)

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
      self.args = parser.parse_args()

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