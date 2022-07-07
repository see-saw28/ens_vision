#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Jun 30 17:01:02 2022

@author: student
"""
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



names = ['_14']

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

for name in names :
    with open(rospack.get_path('ens_vision')+f'/tests/test_aruco{name}.yaml') as file:
        # The FullLoader parameter handles the conversion from YAML
        # scalar values to Python the dictionary format
        param = yaml.load(file, Loader=yaml.FullLoader)
        label = driving_mode_dict[param['driving_mode']] + f" at {param['speed']}m/s"
        labels.append(label)
        traj_name=param['path_filename']
        traj = path_tools.load_path(traj_name, absolute_path=True)
        ref_traj_name=param['ref_traj_name']
        ref_traj = path_tools.mcp_to_path(path_tools.load_mcp(ref_traj_name))
        ref_x, ref_y, ref_yaw = path_tools.path_to_xyyaw(ref_traj)
        x, y, yaw, t = path_tools.path_to_xyyaw(traj, time=True)
        crosstrack, yaw, idx = calc_errors(traj, ref_traj)
print(label)
for name in names :
    with open(rospack.get_path('ens_vision')+f'/tests/test_amcl{name}.yaml') as file:
        # The FullLoader parameter handles the conversion from YAML
        # scalar values to Python the dictionary format
        param = yaml.load(file, Loader=yaml.FullLoader)
        label = driving_mode_dict[param['driving_mode']] #+ f" at {param['speed']}m/s"
        labels.append(label)
        traj_name=param['path_filename']
        traj = path_tools.load_path(traj_name, absolute_path=True)
        ref_x, ref_y, ref_yaw = path_tools.path_to_xyyaw(ref_traj)
        x_a, y_a, yaw_a, t_a = path_tools.path_to_xyyaw(traj, time=True)
        crosstrack_a, yaw_a, idx_a = calc_errors(traj, ref_traj)


#%% AFFICHAGE
n = 150

plt.close('all')

# Compute gobal error
localization_error = []
for i in range(len(x)):
    localization_error.append(np.sqrt((y[i]-y_a[i])**2+ (x[i]-x_a[i])**2))

# Compute crosstrack error and alongtrack error
index = []
loca_crosstrack_errors = []
loca_alongtrack_errors = []
loca_time_delay = []
for i in range(len(idx)) :
    d = np.abs(idx[i]-idx_a[max(0,i-5):min(i+5,len(idx))]).argmin()
    index.append(d+max(0,i-5))
    loca_crosstrack_errors.append(abs(np.sqrt((x[i]-x_a[d+max(0,i-5)])**2+(y[i]-y_a[d+max(0,i-5)])**2)))
    loca_alongtrack_errors.append(np.sqrt(localization_error[i]**2-loca_crosstrack_errors[i]**2))
    loca_time_delay.append(t_a[d+max(0,i-5)]-t[i])

#PLot crosstrack error of each measure
fig, ax = plt.subplots(1,1)
for i in range(n):
    ax.plot([y[i],ref_y[idx[i]]], [x[i],ref_x[idx[i]]], 'b-')

for i in range(n):
    ax.plot([y_a[i],ref_y[idx_a[i]]], [x_a[i],ref_x[idx_a[i]]], 'g-')

ax.plot(y[0:n],x[0:n],'.')
ax.plot(y_a[0:n],x_a[0:n],'.')
ax.plot(ref_y,ref_x,'.')
ax.set_title('Crosstrack error')
#Plot global error
fig, ax1 = plt.subplots(1,1)

for i in range(n):
    ax1.plot([y[i],y_a[i]], [-x[i],-x_a[i]], '-')
    # ax.text(y[i]+i%2*0.02,-x[i],f'{t[i]-t[0]:.3f}')


ax1.plot(y[0:n],-np.array(x[0:n]),'.', label='aruco')
ax1.plot(y_a[0:n],-np.array(x_a[0:n]),'.',label='AMCL')
ax1.legend()
ax1.set_title('Global localization error')

#Plot crosstrack error
fig, ax2 = plt.subplots(1,1)

for i in range(n):
    ax2.plot([y[i],y_a[index[i]]], [-x[i],-x_a[index[i]]], '-')
    # ax.text(y[i]+i%2*0.02,-x[i],f'{t[i]-t[0]:.3f}')


ax2.plot(y[0:n],-np.array(x[0:n]),'.', label='aruco')
ax2.plot(y_a[0:n],-np.array(x_a[0:n]),'.',label='AMCL')
ax2.legend()
ax2.set_title('Crosstrack localization error')




#Box plot
fig, [ax3,ax4] = plt.subplots(1,2)
ax3.boxplot([localization_error,loca_crosstrack_errors, loca_crosstrack_errors], labels=['Global error', 'Crosstrack error', 'Alongtrack error'] ,showfliers=True, showmeans=True, meanline=True, whis=1.5)
ax3.set_title('Localization error (meter)')


ax4.boxplot(loca_time_delay, labels=['Global time delay'] ,showfliers=True, showmeans=True, meanline=True, whis=1.5)
ax4.set_title('Localization delay (second)')



