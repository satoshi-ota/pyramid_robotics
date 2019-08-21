#!/usr/bin/env python
# -*- coding: utf-8 -*-

from jinja2 import Environment, FileSystemLoader
import sys

import csv
import subprocess

args = sys.argv

env = Environment(loader=FileSystemLoader('../urdf/jinja/', encoding='utf8'))
tpl = env.get_template('massless_rigid_tether.xacro')

urdf = tpl.render()

tmpfile = open("../urdf/tether.xacro", 'w') #書き込みモードで開く
tmpfile.write(urdf.encode('utf-8'))
tmpfile.close()
