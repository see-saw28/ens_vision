#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Jun 23 11:31:51 2022

@author: student
"""
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Jun 22 13:39:48 2022

@author: student
"""
import path_tools
import matplotlib.pyplot as plt
import numpy as np
import rospkg
rospack = rospkg.RosPack()
import yaml

plt.close('all')

names = ['test','test_1', 'test_3','test_6']

driving_mode_dict = {0:'MANUAL',1:'Pure Pursuit',2:'Stanley Controller',3:'LCS',4:'DWA',5:'MOVE BASE',6:'FOLLOW THE GAP'}

crosstracks = []
yaws = []
labels = []

for name in names :
    with open(rospack.get_path('ens_vision')+f'/tests/{name}.yaml') as file:
        # The FullLoader parameter handles the conversion from YAML
        # scalar values to Python the dictionary format
        param = yaml.load(file, Loader=yaml.FullLoader)
        label = driving_mode_dict[param['driving_mode']] #+ f" at {param['speed']}m/s"
        labels.append(label)
        error_name=param['error_filename'].split('/')[-1].split('.')[0]
        crosstrack, yaw = path_tools.load_error(error_name, absolute=True, test=True)
        crosstracks.append(crosstrack)
        yaws.append(yaw)


fig, [ax1,ax2] = plt.subplots(1,2)
ax1.set_title('Crosstrack error (meter)')
ct_dict = ax1.boxplot(crosstracks, labels=labels, showmeans=True, meanline=True)
ax2.set_title('Yaw error (radian)')
yaw_dict = ax2.boxplot(yaws, labels = labels,showfliers=True, showmeans=True, meanline=True)