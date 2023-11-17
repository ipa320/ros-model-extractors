#!/usr/bin/env python
import sys
import os
import subprocess
from datetime import datetime
import shutil
import glob

from ros2runner import ros2Runner

# Arguments:
#   1: Package name or Git repository name
#   2: Node name or launch file name or '--all' to analyse all the available nodes 
#   3: Type of the request: either 'launch' or 'node'
#   4: Path to the folder where the resulting model files should be stored
#   5: Path to the ROS workspace 
#   6: Http address links of the Git repositories (to indicate the branch put the name of the repository between quotes and add the suffix -b *banch_name*, for example "https://github.com/ipa320/ros-model-extractors b main"

def main():

    CLI_args = sys.argv[1:]

    if len(CLI_args) < 6:
        print(f'ERROR: At least 6 arguments expected, only {len(CLI_args)} are given.')
        sys.exit()

    pkgName = sys.argv[1] 
    NodeName = sys.argv[2]
    typeOfRequest = sys.argv[3]
    pathToOutput =  sys.argv[4] 
    pathToROSws = sys.argv[5] 
    gitRepo = sys.argv[6]

    ros2Extractor = ros2Runner(inputPkgName=pkgName,
                        inputNodeName=NodeName,
                        typeOfRequest=typeOfRequest,
                        pathToROSws=pathToROSws,
                        gitRepo=gitRepo,
                        outputDir=pathToOutput)
    ros2Extractor.extractorRun(True)
   

if __name__ == "__main__":
    main()