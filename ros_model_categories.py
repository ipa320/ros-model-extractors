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
                print(category['id'])
                links = func(category)
                for a in links:
                    repo = wr.extract_repo_from_wiki(a['href'], 'melodic')
                    if type(repo) == list and repo[0] not in repos and 'github.com' in repo[0]:
                        for pkg in ec.get_package_names_in_repo(repo[0], repo[1]):
                            ec.extract_component(repo[0], pkg, repo[1], 'melodic', category['id'])

                        repos.append(repo[0])
            print('')


if __name__ == '__main__':
    extract_repos_from_meta_page('imu_drivers', 'h2', _func1)
    # extract_repos_from_meta_page('Sensors', 'h3', _func2)
