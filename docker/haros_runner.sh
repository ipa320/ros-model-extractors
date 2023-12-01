#!/bin/bash

# Arguments:
#   1: Package name
#   2: Node name or launch file name or '--all' to analyse all the available nodes 
#   3: Type of the request: either 'launch' or 'node'
#   4: Path to the folder where the resulting model files should be stored
#   5: Path to the ROS workspace 
#   (optional) from 6: Http address links of the Git repositories (to indicate the branch put the name of the repository between quotes and add the suffix -b *banch_name*, for example "https://github.com/ipa320/ros-model-extractors b main" 
# Returns:
#   (None)

# scripts_dir="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

number_of_args=$#
number_of_repos=$((number_of_args-5))

for repo in "${@:6:$number_of_repos}"
do
  cd "${5}"/src
  git clone $repo
done

cd "${5}"
model_repo=$(echo "${6}" | sed 's/ .*//')

if [ -d "${5}/devel" ]; then
  source devel/setup.bash
else
  source install/setup.bash
fi


echo ""
echo "## Install ROS pkgs dependencies ##"
if [ -n $ROS_VERSION ]
then
  if [[ $ROS_VERSION == "1" ]]
  then
    source devel/setup.bash
    rosdep install -y -i -r --from-path src
    catkin_make -DCMAKE_EXPORT_COMPILE_COMMANDS=1
  elif [[ $ROS_VERSION == "2" ]]
  then
    source install/setup.bash
    rosdep install -y -i -r --from-path src
    colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=1 --no-warn-unused-cli
    source ${5}/install/setup.bash
    colcon list > /tmp/colcon_list.txt
    path_to_src_code=$(cat /tmp/colcon_list.txt |  grep "^$1" | awk '{ print $2}')
    if [ -z "$path_to_src_code" ]; then
      echo "** ERROR: Package ${1} not found in the workspace **"
      exit
    fi
    path_to_src_code="${5}/$path_to_src_code"
  else
    echo "ROS version not supported"
    exit
  fi
else
  echo "ROS installation not found"
fi

if [ -n $ROS_DISTRO ]
then
  if [[ $ROS_DISTRO == "humble" ]]
  then
    clang_version=14
  else
    clang_version=10
  fi
fi

echo ""

#tree ${5}

echo "## Init HAROS ##"
mkdir -p "${4}"
haros init

echo ""
echo "## Call the HAROS plugin to extract the ros-models ##"
if [ -n $PYTHON_VERSION ]
then
  if [[ $PYTHON_VERSION == "2" ]]
  then
    if [ "${2}" = "--all" ]
    then
      python /ros_model_extractor.py --clang-version $clang_version --package "$1" --"${3}" --model-path "${4}" --ws "${5}" --repo $model_repo -a>> ${4}/extractor.log
    else
      python /ros_model_extractor.py --clang-version $clang_version --package "$1" --name "$2" --"${3}" --model-path "${4}" --ws "${5}" --repo $model_repo>> ${4}/extractor.log
    fi
  elif [[ $PYTHON_VERSION == "3" ]]
  then
    if [ "${2}" = "--all" ]
    then
      python3 /ros_code_analysis/ros_model_extractor.py --clang-version $clang_version --package "$1" --"${3}" --model-path "${4}" --ws "${5}" --path-to-src "$path_to_src_code" --repo $model_repo -a >> ${4}/extractor.log
    else
      python3 /ros_code_analysis/ros_model_extractor.py --clang-version $clang_version --package "$1" --name "$2" --"${3}" --model-path "${4}" --ws "${5}" --path-to-src "$path_to_src_code" --repo $model_repo>> ${4}/extractor.log
    fi
  else
    echo "Python version not supported"
    exit
  fi
else
  echo "Python setup not found"
fi


echo "~~~~~~~~~~~"
echo "Extraction finished. See the following report:"
cat ${4}/extractor.log
echo "~~~~~~~~~~~"


#echo "~~~~~~~~~~~"
#echo "Compile commands file:"
#cat ${5}/build/compile_commands.json
#echo "~~~~~~~~~~~"

echo "###########"
for generated_model in "${4}"/*.ros2
do
echo "~~~~~~~~~~~"
echo "Print of the model: $generated_model:"
echo "~~~~~~~~~~~"
cat $generated_model
echo ""
echo "~~~~~~~~~~~"
echo "###########"
done

## Clean and finish
#rm -rf ${5}/src/*
