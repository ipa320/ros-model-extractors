#!/usr/bin/env python

import urllib.request
import bs4

import wiki_reader as wr
import extract_components as ec


def _func1(elem):
    return elem.parent.findAll('a')

def _func2(elem):
    return elem.find_next_siblings('ul')[0].findAll('a')

def extract_repos_from_meta_page(page, elem, func):
    soup = wr.read_wiki_page(page)
    repos = list()

    if soup is not None:
        div = soup.find("div", attrs={"class": "table-of-contents"})
        links = div.findAll("a")

        for a in links:
            category = soup.find(elem, id=a['href'].replace('#', ''))
            if category is not None:
                links = func(category)
                for a in links:
                    repo = wr.extract_repo_from_wiki(a['href'], 'melodic')
                    if type(repo) == list and repo[0] not in repos and 'github.com' in repo[0]:
                        for pkg in ec.get_package_names_in_repo(repo[0], repo[1]):
                            ec.extract_component(repo[0], pkg, repo[1], 'melodic', category['id'])

                        repos.append(repo[0])
            print('')


def extract_repos_from_rosi(url):
    soup = wr.read_url(url)
    repos = list()

    if soup is not None:
        links = soup.findAll("a")
        for link in links:
            if link.text == 'Link to ROS Driver':
                git = None
                branch = None
                href = link['href']
                if 'github.com' in href and href not in repos:
                    git = href
                    repos.append(href)
                elif 'wiki.ros.org' in href:
                    repo = wr.extract_repo_from_wiki(href, 'melodic', abs=True)
                    if type(repo) == list and repo[0] not in repos and 'github.com' in repo[0]:
                        git = repo[0]
                        branch = repo[1]
                        repos.append(repo[0])
                if git is not None:
                    pkgs, branch = ec.get_package_names_in_repo(git, branch)
                    if pkgs is not None and branch is not None:
                        for pkg in pkgs:
                            ec.extract_component(git, pkg, branch, 'melodic', '3D_Sensors_Range_finders_RGB-D_cameras')


if __name__ == '__main__':
    # extract_repos_from_meta_page('imu_drivers', 'h2', _func1)
    # extract_repos_from_meta_page('Sensors', 'h3', _func2)
    extract_repos_from_rosi('https://rosindustrial.org/3d-camera-survey')
