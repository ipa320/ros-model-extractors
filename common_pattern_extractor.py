#!/usr/bin/env python

from ros_model_parser.rosmodel_parser import RosModelParser
from ros_model_parser.model_comparator import extract_common_ros
import ros_metamodels.ros_metamodel_core as rosmodel
import ros_model_generator.rosmodel_generator as model_generator
import os.path
from os import listdir
import sys


def extract_common_patterns(model_files):
    set_common_models = set()
    for i in range(len(model_files)):
        for j in range(i + 1, len(model_files)):
            model_a = RosModelParser(model_files[i], isFile=True).parse()
            model_b = RosModelParser(model_files[j], isFile=True).parse()
            res = extract_common_ros(model_a, model_b)
            if res[0] or res[1] or res[2] or res[3] or res[4] or res[5]:
                print(res)
                set_common_models.add(res)

    return set_common_models

def generate_model(model_tuple, pkg_name, artifact_name, model_path):
    ros_node = rosmodel.Node(artifact_name)

    # these tuples should either be named (dict) or
    # the function has to be moved to ros_model_parser
    is_empty = True
    if model_tuple[0]:
        is_empty = False
        for pub in model_tuple[0]:
            ros_node.add_publisher(pub[0], pub[1])

    if model_tuple[1]:
        is_empty = False
        for sub in model_tuple[1]:
            ros_node.add_subscriber(sub[0], sub[1])

    if model_tuple[2]:
        is_empty = False
        for srv in model_tuple[2]:
            ros_node.add_service_server(srv[0], srv[1])

    if model_tuple[3]:
        is_empty = False
        for srv in model_tuple[3]:
            ros_node.add_service_client(srv[0], srv[1])

    if model_tuple[4]:
        is_empty = False
        for act in model_tuple[4]:
            ros_node.add_action_server(act[0], act[1])

    if model_tuple[5]:
        is_empty = False
        for act in model_tuple[5]:
            ros_node.add_action_client(act[0], act[1])

    if is_empty:
        return

    model_file = os.path.join(model_path, pkg_name + ".ros")

    model_gen = model_generator.RosModelGenerator()
    model_gen.create_model_from_node(pkg_name, artifact_name, ros_node)
    model_gen.generate_ros_model(model_file)

if __name__=='__main__':
    model_dir = sys.argv[1]
    model_files = [os.path.join(model_dir, f) for f in listdir(model_dir) if os.path.isfile(os.path.join(model_dir, f))]

    common_models = extract_common_patterns(model_files)
    print("Extracted " + str(len(common_models)) + " common patterns")

    pattern_path = os.path.join(model_dir, 'common')
    if not os.path.exists(pattern_path):
        os.makedirs(pattern_path)

    cnt = 1
    for model in common_models:
        name = "pattern_" + str(cnt)
        generate_model(model, name, name, pattern_path)
        cnt = cnt + 1
