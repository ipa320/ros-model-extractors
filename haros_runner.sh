#!/bin/bash

# Arguments:
#   1: Package name
#   2: Node name or launch file name
#   3: Type of the request: either 'launch' or 'node'
#   4: Path to the folder where the resulting model files should be stored
#   5: Path to the ROS workspace 
#   (optional) 6: Http address link of the Git repository 
#   (optional) 7: Branch for the Git repository 
# Returns:
#   (None)

# scripts_dir="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

if [ "$#" -eq 6 ]; then
   cd "${5}"/src
   git clone "${6}"
   cd "${5}"
fi

if [ "$#" -eq 7 ]; then
   cd "${5}"/src
   git clone "${6}" -b "${7}"
   cd "${5}"
fi

cd "${5}"

echo ""
echo "## Install ROS pkgs dependencies ##"
if [ -n $ROS_VERSION ]
then
  if [ $ROS_VERSION == "1" ]
  then
    source devel/setup.bash
    rosdep install -y -i -r --from-path src
    catkin_make -DCMAKE_EXPORT_COMPILE_COMMANDS=1
  elif [ $ROS_VERSION == "2" ]
  then
    source install/setup.bash
    rosdep install -y -i -r --from-path src
    colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
    source /root/ws/install/setup.bash
    colcon list > /tmp/colcon_list.txt
    path_to_src_code=$(cat /tmp/colcon_list.txt |  grep "^$1" | awk '{ print $2}')
    if [ -z "$path_to_src_code" ]; then
      echo "** ERROR: Package ${1} not found in the workspace **"
      exit
    fi
    path_to_src_code="/root/ws/$path_to_src_code"
  else
    echo "ROS version not supported"
    exit
  fi
else
  echo "ROS installation not found"
fi

echo ""

#tree ${5}

echo "## Init HAROS ##"

haros init

echo ""
echo "## Call the HAROS plugin to extract the ros-models ##"
if [ -n $PYTHON_VERSION ]
then
  if [ $PYTHON_VERSION == "2" ]
  then
    python /ros_model_extractor.py --package "$1" --name "$2" --"${3}" --model-path "${4}" --ws "${5}">> extractor.log
    cat extractor.log
  elif [ $PYTHON_VERSION == "3" ]
  then
    python3 /ros_model_extractor.py --package "$1" --name "$2" --"${3}" --model-path "${4}" --ws "${5}" --path-to-src "$path_to_src_code">> extractor.log
    cat extractor.log 
  else
    echo "Python version not supported"
    exit
  fi
else
  echo "Python setup not found"
fi

echo "###########"
echo "~~~~~~~~~~~"
echo "Print of the model: $2.ros:"
echo "~~~~~~~~~~~"
cat "${4}"/"$2".ros
echo ""
echo "~~~~~~~~~~~"
echo "###########"
