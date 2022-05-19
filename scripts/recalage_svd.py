# -*- coding: utf-8 -*-
"""
Created on Thu May 19 22:50:21 2022

https://hal.inria.fr/inria-00073710/document

@author: paulg
"""
import numpy as np


di=np.array([[11.157,-0.089357,0.80816],[13.368, 1.6957, 2.7892],[12.681,2.2539,5.0729]])
d=di.reshape((3,3,1))
# si=np.array([[1,3,2],[1, 1, 3],[1,1,1]])
si=np.array([[1,0,1],[3, 2, 3],[2,3,5]])
s=si.reshape((3,3,1))

# di=np.array([[10,0,1],[12, 2, 1],[11,3,1]])
# d=di.reshape((3,3,1))
# # si=np.array([[1,3,2],[1, 1, 3],[1,1,1]])
# si=np.array([[1,0,1],[3, 2, 1],[2,3,1]])
# s=si.reshape((3,3,1))

s_b=np.mean(s,axis=0)
d_b=np.mean(d,axis=0)

d_c=d-d_b
s_c=s-s_b

H=np.zeros((len(s),len(s)))
for i in range(len(s)):
    H+=np.dot(s_c[i],np.transpose(d_c[i]))
    
# H=H/len(s)

# H=np.dot(s_c.reshape((3,3)),np.transpose(d_c.reshape(3,3)))

U,S,V = np.linalg.svd(H)

V=np.transpose(V)


R=np.dot(V,np.transpose(U))
# print(R)
if(np.linalg.det(R)<0):
    # print(V)
    V_p=np.copy(V)
    V_p[:,2]=-V[:,2]
    # print(V_p)
    R=np.dot(V_p,np.transpose(U))


T=d_b-np.dot(R,s_b)

df=np.transpose(np.dot(R,np.transpose(si))+T)