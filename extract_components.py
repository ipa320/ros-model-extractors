#!/usr/bin/env python

import json
import requests
import subprocess
import xml.etree.ElementTree as ET


def _get_user_repo(url):
    res = url.split('/')
    return res[-2], res[-1].replace('.git', '')


def get_package_names_in_repo(url, branch):
    pkg_names = []
    user, repo = _get_user_repo(url)

    git_api = 'https://api.github.com/repos'

    if branch is None:
        api_url = git_api + '/{}/{}'.format(user, repo)
        try:
            branch = requests.get(api_url, auth=(my_username, my_token)).json()['default_branch']
        except KeyError as e:
            print(e)
            return None, None

    api_url = git_api + '/{}/{}/git/trees/{}?recursive=1'.format(user, repo, branch)

    req = requests.get(api_url, auth=(my_username, my_token))
    pkgs = req.json()['tree']
    for pkg in pkgs:
        if pkg['path'].endswith('package.xml'):
            xml = 'https://raw.githubusercontent.com/' + user + '/' + repo + '/' + branch + '/' + pkg['path']
            r = requests.get(xml)
            tree = ET.fromstring(r.content)
            for child in tree.iter('name'):
                url = "https://github.com/" + user + '/' + repo + '.git'
                pkg_names.append(child.text)

    return pkg_names, branch


def extract_component(repo, pkg, branch, distro, category=''):
    cmd='docker-compose run '+distro+' /haros_runner.sh '+pkg+' --all node /home/extractor/results/'+category+' \
        /home/extractor/ws "'+repo+' -b '+branch+'"'
    print(f"Calling the command ",cmd,"\n")
    output,error = subprocess.Popen(cmd, universal_newlines=True, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE).communicate()

    print(output)


if __name__ == '__main__':
    repo = 'https://github.com/ros-simulation/gazebo_ros_pkgs'
    branch = 'melodic-devel'
    distro = 'melodic'
    pkgs = get_package_names_in_repo(repo, branch)
    for pkg in pkgs:
        extract_component(repo, pkg, branch, distro, 'simulation/gazebo')
