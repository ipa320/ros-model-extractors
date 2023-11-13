#!/usr/bin/env python
import sys
import os
import subprocess
from datetime import datetime
import shutil

import glob


class fileHandling:
    def __init__(self):
        pass
   
    @staticmethod
    def wrtieToFile(fileNameWithPath, contentToWrite):
        with open(fileNameWithPath, "w") as file:
                file.write(contentToWrite)
        print("-----Write to {} completed-----".format(fileNameWithPath))
       
    @staticmethod
    def showFileContent(fileNameWithPath):
       
        print("-----Show the ROS model of {}".format(fileNameWithPath))
        with open(fileNameWithPath,'r') as file:
            print(file.read())
           
    @staticmethod
    def filterFileNamesByExtension(rootDir, extensionToSearch):
       
        fileList = glob.glob(os.path.join(rootDir, '**', "*.{}".format(extensionToSearch)), recursive=True)
       
        return fileList