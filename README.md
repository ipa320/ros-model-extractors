# ros-model-extractors

[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)

[![MELODIC build status](https://github.com/ipa320/ros-model-extractors/actions/workflows/build_melodic.yml/badge.svg)](https://github.com/ipa320/ros-model-extractors/actions/workflows/build_melodic.yml)
[![NOETIC ros-model-extractors](https://github.com/ipa320/ros-model-extractors/actions/workflows/build_noetic.yml/badge.svg)](https://github.com/ipa320/ros-model-extractors/actions/workflows/build_noetic.yml)
[![FOXY build status](https://github.com/ipa320/ros-model-extractors/actions/workflows/build_foxy.yml/badge.svg)](https://github.com/ipa320/ros-model-extractors/actions/workflows/build_foxy.yml)


Technical Maintainer: [**ipa-nhg**](https://github.com/ipa-nhg/) (**Nadia Hammoudeh Garcia**, **Fraunhofer IPA**) - **nadia.hammoudeh.garcia@ipa.fraunhofer.de**

This repository contains the HAROS framework plugin to automatically generate models according to the DSLs defined for RosModel.

HAROS is a framework for static code analysis, in this case our plugin uses the result of the analysis to find the communication interfaces of each ROS node and directly translate it into our model structure, which can be validated or used to compose it with other nodes forming a system,

This package also contains a set of ROS containers where you can easily do the analysis for different distros without the need to install locally the necessary software. These containers take as input argument the URL of the GitHub repository that contains the code.

As related work you can read the following paper: [Bootstrapping MDE development from ROS manual code: Part 2—Model generation and leveraging models at runtime](https://link.springer.com/article/10.1007/s10270-021-00873-2?wt_mc=Internal.Event.1.SEM.ArticleAuthorOnlineFirst&utm_source=ArticleAuthorOnlineFirst&utm_medium=email&utm_content=AA_en_06082018&ArticleAuthorOnlineFirst_20210420) 


### HowTo Use the docker container to run the ros-model plugin for HAROS

Install docker https://docs.docker.com/install/linux/docker-ce/ubuntu/

Build the HAROS docker image, for your desired ROS distro version:

```
cd path-to-ros-model-extractors-repo
[sudo] docker build --tag=haros_ROSDISTRO -f ROSDISTRO/Dockerfile .
```

Call the ros-model extractor plugin, remember you have to also clone the repository to be analysed:

```
[sudo] docker run -it haros_ROSDISTRO:latest /haros_runner.sh *package_name* *node_name* *type* *path_to_resulted_model* *workspace_path* "*github_repositoryA -b branch*" "*github_repositoryB*"...
```

Where the type is the type of the analysis, now only the option "node" analysis is supported. Soon the option "launch" will be also available. The default configuration of our infrastructure expects the current path for the resulted model and the folder "/root/ws" for the workspace. The argument branch is optional.

```
[sudo] docker run -it haros_ROSDISTRO:latest /haros_runner.sh *package_name* *node_name* *type* . /root/ws "*github_repositoryA -b branch*" "*github_repositoryB*"...
```

Additionally, the analysis offers the option to analyze all the nodes of a package recursively:

```
[sudo] docker run -it haros_ROSDISTRO:latest /haros_runner.sh *package_name* --all *type* . /root/ws "*github_repositoryA -b branch*" "*github_repositoryB*"...
```

Please check the available examples for the supported distros:

- [ROS1 melodic](melodic/README.md)
- [ROS1 noetic](noetic/README.md)
- [ROS2 foxy](foxy/README.md)

ToDo:
 - Extractor of interfaces types (msgs, srvs and actions)
 - Parser for launch files and analysis of the full system 

