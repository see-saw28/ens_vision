#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Jun 22 13:39:48 2022

@author: student
"""
import path_tools
import matplotlib.pyplot as plt
import numpy as np


names = ['error_4','error_5']

errors = np.array([path_tools.load_error(name, absolute=True) for name in names])

crosstrack = errors[:,0]
yaw = errors[:,1]

fig, [ax1,ax2] = plt.subplots(1,2)
ax1.set_title('Crosstrack error (meter)')
ct_dict = ax1.boxplot(crosstrack, labels=names)
ax2.set_title('Yaw error (radian)')
yaw_dict = ax2.boxplot(yaw, labels = names,showfliers=False)