#!/usr/bin/python3

import rosinstall_generator.distro as distro

distros = ['noetic', 'melodic', 'lunar', 'kinetic', 'jade', 'indigo', 'hydro', 'groovy']

def get_packages_repo(distro_name): 
    ros_distro = distro.get_distro(distro_name)
    pkg_names = distro.get_package_names(ros_distro)
    for pkg_name in pkg_names[0]:
        pkg = ros_distro.release_packages[pkg_name]
        repo = ros_distro.repositories[pkg.repository_name].source_repository

        try:
            yield [pkg_name, repo.url, repo.version]
        except AttributeError as exc:
            print(exc)

def loop_distros():
    pkg_list = list()
    for distro_name in distros:
        pkgs = get_packages_repo(distro_name)
        try:
            for pkg in pkgs:
                if not any(pkg[0] in pkg_info for pkg_info in pkg_list):
                    pkg.append(distro_name)
                    pkg_list.append(pkg)
        except Exception as exc:
            print(exc)
    
    return pkg_list

if __name__ == '__main__':
    pkg_list = loop_distros()
    print(pkg_list)