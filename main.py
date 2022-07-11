import numpy as np
import scipy as sp
import math
import control as ct
import matplotlib.pyplot as plt

#parameters
g=9.81 #gravity [kg*m/s^2]
m=1 #mass [kg]
l=0.5 #length [m]

deg2rad=np.pi/180
rad2deg=180/np.pi

A=np.array([[0, 1], [-g/l, 0]])
B=np.array([[0], [1/(m*pow(l, 2))]])
C=np.array([[1], [0]])
D=0;
R=np.ones((1, 1), dtype=float)
Q=np.ones((2, 2), dtype=float)

K, S, E=ct.lqr(A, B, Q, R)

theta_start=10*deg2rad
theta_stop=0.
for i in range(0, 1000):
    print(f"iteration={i}")