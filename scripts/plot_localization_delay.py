#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Jun 28 09:29:26 2022

@author: student
"""
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

#PP
names = ['_11', '_12','_13','_14','_15']

#SC cannot achieve 5 laps at 1.5m/s
names += ['_16','_17','_18']

#LCS
names += ['_19','_20','_21','_22']

#DWA
names += ['_23','_24','_25','_26']

# #0.75
# names = ['_11','_16','_19','_23']

# #1.0
# names = ['_12','_17','_20','_24']

# #1.25
# names = ['_13','_18','_21','_25']

# #1.5
# names = ['_15','_14','_22','_26']

# Aruco vs AMCL localization
# names = ['_29', '_14']

#FTG
names += ['_30', '_31', '_32', '_33', '_34']

driving_mode_dict = {0:'MANUAL',1:'Pure Pursuit',2:'Stanley Controller',3:'LCS',4:'DWA',5:'MOVE BASE',6:'FOLLOW THE GAP'}

crosstracks = []
yaws = []
labels = []

crosstracks_amcl = []
yaws_amcl = []
labels_amcl = []


def normalize_angle(angle):
    # adapted from: https://github.com/AtsushiSakai/PythonRobotics/tree/master/PathTracking/stanley_controller

    while angle > np.pi:
        angle -= 2.0 * np.pi

    while angle < -np.pi:
        angle += 2.0 * np.pi

    return angle

def calc_error(x, y, yaw, course_x, course_y, course_yaw):
    # adapted from: https://github.com/AtsushiSakai/PythonRobotics/tree/master/PathTracking/stanley_controller



    # Search nearest point index
    dx = [x - icx for icx in course_x]
    dy = [y - icy for icy in course_y]
    d = [np.sqrt(idx ** 2 + idy ** 2) for (idx, idy) in zip(dx, dy)]

    error_front_axle = min(d)
    target_idx = d.index(error_front_axle)
    # print(target_idx)


    error_yaw = normalize_angle(course_yaw[target_idx] - yaw)


    return error_front_axle, error_yaw, target_idx

def calc_errors(traj, ref_path):
    ref_x, ref_y, ref_yaw = path_tools.path_to_xyyaw(ref_path)
    x, y, yaw = path_tools.path_to_xyyaw(traj)



    crosstrack_errors = np.zeros((len(x),))
    yaw_errors = np.zeros((len(x),))
    idx_errors = np.zeros((len(x),),dtype=np.int32)

    for i in range(len(x)):
        crosstrack_error, yaw_error, target_idx = calc_error(x[i], y[i], yaw[i], ref_x, ref_y, ref_yaw)
        crosstrack_errors[i] = abs(crosstrack_error)
        yaw_errors[i] = abs(yaw_error)
        idx_errors[i] = target_idx

    return crosstrack_errors, yaw_errors, idx_errors

loca_time_delays = []
loca_mean_time_delays = []
labels = []

for name in names :
    with open(rospack.get_path('ens_vision')+f'/tests/test_aruco{name}.yaml') as file:
        # The FullLoader parameter handles the conversion from YAML
        # scalar values to Python the dictionary format
        param = yaml.load(file, Loader=yaml.FullLoader)
        label = driving_mode_dict[param['driving_mode']] + f" at {param['speed']}m/s"
        labels.append(label)
        traj_name=param['path_filename']
        traj = path_tools.load_path(traj_name, absolute_path=True)
        x, y, yaw, t = path_tools.path_to_xyyaw(traj,time=True)
        ref_traj_name=param['ref_traj_name']
        ref_traj = path_tools.mcp_to_path(path_tools.load_mcp(ref_traj_name))
        crosstrack, yaw, idx = calc_errors(traj, ref_traj)

    with open(rospack.get_path('ens_vision')+f'/tests/test_amcl{name}.yaml') as file:
        # The FullLoader parameter handles the conversion from YAML
        # scalar values to Python the dictionary format
        param = yaml.load(file, Loader=yaml.FullLoader)

        traj_name=param['path_filename']
        traj_amcl = path_tools.load_path(traj_name, absolute_path=True)
        x_a, y_a, yaw_a, t_a = path_tools.path_to_xyyaw(traj_amcl,time=True)
        crosstrack_a, yaw_a, idx_a = calc_errors(traj_amcl, ref_traj)

    index = []
    loca_crosstrack_errors = []
    loca_alongtrack_errors = []
    loca_time_delay = []
    for i in range(len(idx)) :
        d = np.abs(idx[i]-idx_a[max(0,i-5):min(i+5,len(idx))]).argmin()
        index.append(d+max(0,i-5))
        loca_time_delay.append(t_a[d+max(0,i-5)]-t[i])

    loca_time_delays.append(np.array(loca_time_delay))
    loca_mean_time_delays.append(np.mean(loca_time_delay))

print(label)

#%% AFFICHAGE
fig, ax = plt.subplots(1,1)

n = 300

for i in range(n):
    ax.plot([y[i],y_a[i]], [-x[i],-x_a[i]], '-')
    # ax.text(y[i]+i%2*0.02,-x[i],f'{t[i]-t[0]:.3f}')


ax.plot(y[0:n],-np.array(x[0:n]),'.', label='aruco')
ax.plot(y_a[0:n],-np.array(x_a[0:n]),'.',label='AMCL')
ax.legend()

localization_error = []
for i in range(len(x)):
    localization_error.append(np.sqrt((y[i]-y_a[i])**2+ (x[i]-x_a[i])**2))


fig, ax1 = plt.subplots(1,1)
ax1.boxplot(loca_time_delays ,showfliers=True, showmeans=True, meanline=True, whis=1.5)
ax1.set_title('Localization delay (second)')
ax1.legend()

fig, ax2 = plt.subplots(1,1)
ax2.boxplot(loca_mean_time_delays,showfliers=True, showmeans=True, meanline=True, whis=1.5)
ax2.set_title('Mean localization delay (second)')
# fig, [[ax1,ax2],[ax3,ax4]] = plt.subplots(2,2)
# ax1.set_title('Crosstrack error (meter)')
# ct_dict = ax1.boxplot(crosstracks, labels=labels, showmeans=True, meanline=True, whis=1.5)
# ax2.set_title('Yaw error (radian)')
# yaw_dict = ax2.boxplot(yaws, labels = labels,showfliers=True, showmeans=True, meanline=True, whis=1.5)

# for name in names :
#     with open(rospack.get_path('ens_vision')+f'/tests/{name}_amcl.yaml') as file:
#         # The FullLoader parameter handles the conversion from YAML
#         # scalar values to Python the dictionary format
#         param = yaml.load(file, Loader=yaml.FullLoader)
#         label = driving_mode_dict[param['driving_mode']] #+ f" at {param['speed']}m/s"
#         labels_amcl.append(label)
#         traj_name=param['path_filename']
#         traj = path_tools.load_path(traj_name, absolute_path=True)
#         ref_traj_name=param['ref_traj_name']
#         ref_traj = path_tools.mcp_to_path(path_tools.load_mcp(ref_traj_name))
#         crosstrack, yaw = calc_errors(traj, ref_traj)
#         crosstracks_amcl.append(crosstrack)
#         yaws_amcl.append(yaw)

# ax3.set_title('Crosstrack error AMCL (meter)')
# ct_dict = ax3.boxplot(crosstracks_amcl, labels=labels_amcl, showmeans=True, meanline=True, whis=1.5)
# ax4.set_title('Yaw error AMCL (radian)')
# yaw_dict = ax4.boxplot(yaws_amcl, labels = labels_amcl,showfliers=True, showmeans=True, meanline=True, whis=1.5)