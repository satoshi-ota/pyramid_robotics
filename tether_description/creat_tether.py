#!/usr/bin/env python
# -*- coding: utf-8 -*-

from jinja2 import Environment, FileSystemLoader
import sys
#CSVモジュール
import csv
import subprocess

args = sys.argv

#テンプレートファイルを指定
env = Environment(loader=FileSystemLoader('./urdf/', encoding='utf8'))
tpl = env.get_template('tether_fragment.xacro')

#テンプレートへの挿入
urdf = tpl.render()

#ファイルへの書き込み
tmpfile = open("./urdf/tether.xacro", 'w') #書き込みモードで開く
tmpfile.write(urdf.encode('utf-8'))
tmpfile.close()
