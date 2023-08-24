import clang.cindex
import os

def check_cursor_kind(cursor, target_kind):
    return cursor.kind == target_kind


def main():
    clang.cindex.Config.set_library_path("/usr/lib/llvm-14/lib")
    #clang.cindex.Config.set_standard_includes("/usr/lib/llvm-14/lib/clang/14.0.0/include")

    index = clang.cindex.Index.create()
    translation_unit = index.parse(None, clang_cmd)

    root_cursor = translation_unit.cursor
    
    # variable_value = clang.cindex.CursorKind.get_all_kinds()
    # file_path = "CursoKindList.txt"  
    # file_path = os.path.join(os.path.dirname( __file__ ), file_path)
    # with open(file_path, "w") as file:
    #     for item in variable_value:
    #         file.write(str(item) + "\n")


    # print(f"Variable value saved to '{file_path}'.")

    targetKindList = [clang.cindex.CursorKind.FUNCTION_DECL,
                        clang.cindex.CursorKind.PARM_DECL,
                        clang.cindex.CursorKind.VAR_DECL,
                        clang.cindex.CursorKind.CXX_METHOD,
                        #clang.cindex.CursorKind.MEMBER_REF,
                        clang.cindex.CursorKind.TYPE_REF,
                        #clang.cindex.CursorKind.CXX_BASE_SPECIFIER,
                        clang.cindex.CursorKind.VARIABLE_REF,
                        clang.cindex.CursorKind.CLASS_TEMPLATE
                        # clang.cindex.CursorKind.CLASS_TEMPLATE_PARTIAL_SPECIALIZATION,
                        # clang.cindex.CursorKind.CONVERSION_FUNCTION,
                        #clang.cindex.CursorKind.TEMPLATE_TYPE_PARAMETER,
                        #clang.cindex.CursorKind.TEMPLATE_NON_TYPE_PARAMETER,
                        #clang.cindex.CursorKind.TEMPLATE_TEMPLATE_PARAMETER,
                        # clang.cindex.CursorKind.FUNCTION_TEMPLATE,
                        # clang.cindex.CursorKind.UNEXPOSED_STMT,
                        # clang.cindex.CursorKind.INIT_LIST_EXPR,
                        #clang.cindex.CursorKind.TEMPLATE_REF
                        # clang.cindex.CursorKind.TYPE_ALIAS_TEMPLATE_DECL,
                        # # clang.cindex.CursorKind.OVERLOADED_DECL_REF,
                        # # clang.cindex.CursorKind.CXX_OVERRIDE_ATTR,
                        # # clang.cindex.CursorKind.OVERLOAD_CANDIDATE
                        # # # clang.cindex.CursorKind.CXX_FINAL_ATTR,
                        # # clang.cindex.CursorKind.LABEL_REF
                        # clang.cindex.CursorKind.DECL_REF_EXPR,
                        # clang.cindex.CursorKind.MEMBER_REF_EXPR
                    ]

    for cursor in root_cursor.walk_preorder()  :

        if not("/home/divya/ros2_ws/" in str(cursor.location)):
            continue

        if cursor.kind in targetKindList:
            # print("{}".format(cursor.spelling))
            if  cursor.kind == clang.cindex.CursorKind.VAR_DECL:
                var_type = cursor.type
                var_canonical = var_type.get_canonical()
                print("Var canonical",var_canonical.spelling)
                if var_type.kind == clang.cindex.TypeKind.POINTER:
                    pointed_to_type = var_type.get_pointee()
                    print(f"Pointed-to Type: {pointed_to_type.spelling}")
            elif cursor.kind == clang.cindex.CursorKind.PARM_DECL:
                param_type = cursor.type
                param_canonical = param_type.get_canonical()
                print("Param canonical",param_type.spelling)
            elif cursor.kind == clang.cindex.CursorKind.VARIABLE_REF:
                var_ref_type = cursor.type
                var_ref_canonical = var_ref_type.get_canonical()
                print("Var Ref canonical",var_ref_type.spelling)

            

            # if  cursor.kind == clang.cindex.CursorKind.PARM_DECL and (len(cursor.spelling) < 1):
            #     print("location: ", str(cursor.location))
            #     print("parm decl found")
            #     print("displayname: ", cursor.displayname)
            #     print("get_included_file: ", cursor.get_included_file)
            #     print("get_usr: ", cursor.get_usr)

clang_cmd = ['-I/usr/lib/llvm-14/lib/clang/14.0.0/include',
                '--driver-mode=g++', '-DDEFAULT_RMW_IMPLEMENTATION=rmw_fastrtps_cpp', 
                '-DQT_CORE_LIB', '-DQT_GUI_LIB', '-DQT_NO_DEBUG', '-DQT_WIDGETS_LIB', 
                '-DRCUTILS_ENABLE_FAULT_INJECTION', '-DROS_PACKAGE_NAME="turtlesim"', 
                '-I/home/divya/ros2_ws/src/ros_tutorials/turtlesim/include', 
                '-I/home/divya/ros2_ws/build/turtlesim/rosidl_generator_cpp', 
                '-isystem', '/usr/include/x86_64-linux-gnu/qt5', '-isystem', 
                '/usr/include/x86_64-linux-gnu/qt5/QtWidgets', '-isystem', 
                '/usr/include/x86_64-linux-gnu/qt5/QtGui', '-isystem', 
                '/usr/include/x86_64-linux-gnu/qt5/QtCore', '-isystem', 
                '/usr/lib/x86_64-linux-gnu/qt5/mkspecs/linux-g++', 
                '-isystem', '/opt/ros/humble/include/ament_index_cpp',
                '-isystem', '/opt/ros/humble/include/geometry_msgs', 
                '-isystem', '/opt/ros/humble/include/rclcpp', 
                '-isystem', '/opt/ros/humble/include/rclcpp_action', 
                '-isystem', '/opt/ros/humble/include/std_msgs',
                '-isystem', '/opt/ros/humble/include/std_srvs', 
                '-isystem', '/opt/ros/humble/include/builtin_interfaces', 
                '-isystem', '/opt/ros/humble/include/rosidl_runtime_c', 
                '-isystem', '/opt/ros/humble/include/rcutils', 
                '-isystem', '/opt/ros/humble/include/rosidl_typesupport_interface', 
                '-isystem', '/opt/ros/humble/include/fastcdr',
                '-isystem', '/opt/ros/humble/include/rosidl_runtime_cpp', 
                '-isystem', '/opt/ros/humble/include/rosidl_typesupport_fastrtps_cpp', 
                '-isystem', '/opt/ros/humble/include/rmw', 
                '-isystem', '/opt/ros/humble/include/rosidl_typesupport_fastrtps_c', 
                '-isystem', '/opt/ros/humble/include/rosidl_typesupport_introspection_c', 
                '-isystem', '/opt/ros/humble/include/rosidl_typesupport_introspection_cpp', 
                '-isystem', '/opt/ros/humble/include/libstatistics_collector', 
                '-isystem', '/opt/ros/humble/include/rcl', '-isystem', '/opt/ros/humble/include/rcl_interfaces', 
                '-isystem', '/opt/ros/humble/include/rcl_logging_interface', 
                '-isystem', '/opt/ros/humble/include/rcl_yaml_param_parser', 
                '-isystem', '/opt/ros/humble/include/libyaml_vendor', 
                '-isystem', '/opt/ros/humble/include/tracetools', 
                '-isystem', '/opt/ros/humble/include/rcpputils', 
                '-isystem', '/opt/ros/humble/include/statistics_msgs', 
                '-isystem', '/opt/ros/humble/include/rosgraph_msgs', 
                '-isystem', '/opt/ros/humble/include/rosidl_typesupport_cpp', 
                '-isystem', '/opt/ros/humble/include/rosidl_typesupport_c', 
                '-isystem', '/opt/ros/humble/include/action_msgs', 
                '-isystem', '/opt/ros/humble/include/unique_identifier_msgs', 
                '-isystem', '/opt/ros/humble/include/rcl_action', '-Wall', '-Wextra', '-Wpedantic', 
                '-fPIC', '-o', 'CMakeFiles/turtlesim_node.dir/src/turtle_frame.cpp.o', '-c', 
                '/home/divya/ros2_ws/src/ros_tutorials/turtlesim/src/turtle_frame.cpp']

if __name__ == "__main__":
    main()

