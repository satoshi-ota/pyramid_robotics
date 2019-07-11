#!/usr/bin/env python
# -*- coding: utf-8 -*-

from jinja2 import Environment, FileSystemLoader
import sys
#CSVモジュール
import csv
import subprocess

args = sys.argv

#テンプレートファイルを指定
env = Environment(loader=FileSystemLoader('./', encoding='utf8'))
tpl = env.get_template('template.xacro')

#CSVファイル読み込み 引数指定がないときはデフォルトでrobot_param.csv
if len(args) > 1 :
  f = open(args[1], 'rb')
else :
  f = open('robot_param2.csv', 'rb')

if len(args) > 2 :
  robot_name = args[2]
elif len(args) > 1 :
  name = args[1].rsplit(".", 1)
  robot_name = name[0]
else :
  robot_name = u"sample_robot"

dataReader = csv.reader(f)

#テンプレートへ挿入するデータの作成
#robot_name = u"six_dof_arm"
sample_list = []
count = 0
for raw in dataReader:
  if count != 0 :
    if 'base' in raw[0] : base_link=raw[0]
    children_num = 0
    if ',' in raw[1] :
      children_num = 1
      children=raw[1].split(",")
      for child in children :
        sample_list.append({'link':raw[0], 'child_link':child, 'link_rpy':raw[2], 'link_xyz':raw[3], 'link_size':raw[4], 'link_color':raw[5], 'link_mass':raw[6],'link_ixx':raw[7], 'link_ixy':raw[8], 'link_ixz':raw[9], 'link_iyy':raw[10], 'link_iyz':raw[11], 'link_izz':raw[12], 'joint_type':raw[13+8*(children_num-1)], 'joint_rpy':raw[14+8*(children_num-1)], 'joint_xyz':raw[15+8*(children_num-1)], 'joint_axis':raw[16+8*(children_num-1)], 'joint_effort':raw[17+8*(children_num-1)], 'joint_lower':raw[18+8*(children_num-1)], 'joint_upper':raw[19+8*(children_num-1)], 'joint_velocity':raw[20+8*(children_num-1)], 'children_num':children_num})
        children_num += 1
    else :
      sample_list.append({'link':raw[0], 'child_link':raw[1], 'link_rpy':raw[2], 'link_xyz':raw[3], 'link_size':raw[4], 'link_color':raw[5], 'link_mass':raw[6],'link_ixx':raw[7], 'link_ixy':raw[8], 'link_ixz':raw[9], 'link_iyy':raw[10], 'link_iyz':raw[11], 'link_izz':raw[12], 'joint_type':raw[13], 'joint_rpy':raw[14], 'joint_xyz':raw[15], 'joint_axis':raw[16], 'joint_effort':raw[17], 'joint_lower':raw[18], 'joint_upper':raw[19], 'joint_velocity':raw[20], 'children_num':children_num})
  count +=1

print "%d links data read" % (count-1)

#テンプレートへの挿入
urdf = tpl.render({'robot_name':robot_name, 'base_link':base_link, 'sample_list':sample_list})

#ファイルへの書き込み
tmpfile = open(robot_name+".xacro", 'w') #書き込みモードで開く
tmpfile.write(urdf.encode('utf-8'))
tmpfile.close()
