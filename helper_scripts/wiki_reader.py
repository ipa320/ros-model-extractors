#!/usr/bin/env python

import urllib.request
import bs4
import re

distros = ['hydro', 'indigo', 'kinetic', 'lunar', 'melodic', 'noetic']

def _extract_repo_from_pkg_info(pkg_info):
   for li in pkg_info:
      if 'Source:' in li.text:
         source = li.text.split()
         url = source[2]
         branch = source[-1].replace(")", '')
         return [url, branch]


def _get_pkg_info(soup, distro, id='package-info'):
   div = None
   idx = distros.index(distro)
   while div == None and idx > 1:
      div = soup.find("div", attrs={"class": "version " + distro})
      idx = distros.index(distro)
      distro = distros[idx - 1]

   pkg_info = []
   if div is not None:  
      pkg_info = div.findAll('li')

   return pkg_info


def read_url(url):
   soup = None
   try:
      with urllib.request.urlopen(url) as f:
         content = f.read()
         soup = bs4.BeautifulSoup(content, features="html.parser")

   except urllib.error.URLError as e:
      print(e.reason)

   return soup


def read_wiki_page(wiki_page):
   wiki_url = 'http://wiki.ros.org.ros.informatik.uni-freiburg.de/' + wiki_page
   return read_url(wiki_url)


def extract_repo_from_wiki(wiki_page, ros_distro, abs=False):
   soup = None
   if abs:
      soup = read_url(wiki_page)
   else:
      soup = read_wiki_page(wiki_page)

   if soup is not None:
      pkg_info = _get_pkg_info(soup, ros_distro)
      repo = _extract_repo_from_pkg_info(pkg_info)
      return repo


if __name__ == '__main__':
   repo = extract_repo_from_wiki('duo3d-driver', 'melodic')
   print(repo)
