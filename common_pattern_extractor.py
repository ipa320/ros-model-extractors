#!/usr/bin/env python

from ros_model_parser.rosmodel_parser import RosModelParser
from ros_model_parser.model_comparator import compare_ros_models
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
            res = compare_ros_models(model_a, model_b)
            if res[0] or res[1]:
                set_common_models.add(res)

    return set_common_models

def generate_model(model_tuple, pkg_name, artifact_name, model_path):
    ros_node = rosmodel.Node(artifact_name)

    is_empty = True
    if model_tuple[0]:
        is_empty = False
        for pub in model_tuple[0]:
            ros_node.add_publisher(pub[0], pub[1])

    if model_tuple[1]:
        is_empty = False
        for sub in model_tuple[1]:
            ros_node.add_subscriber(sub[0], sub[1])

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
        generate_model(model, name, name, '/home/ipa-hsd/projects/kogrob2/ros-models/ros-model-extractors/noetic/common')
        cnt = cnt + 1
