#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt  # MATLAB plotting functions
from control import *  # MATLAB-like functions

def compute_input(X):
    r = 0.12            # arm length (meter)
    M = 0.021197        # mass of arm (kg)

    m = 0.029804        # mass of pendulum link (kg)
    L = 0.30            # pendulum length (meter)

    g = 9.8

    a1 = (1.0/3.0)*M*r*r + m*r*r;
    a2 = (1.0/2.0)*m*r*L;
    a3 = (1.0/3.0)*m*L*L
    a4 = (1.0/2.0)*m*g*L

    a23 = a2*a4/(a2*a2 - a1*a3);
    a43 = -a1*a4/(a2*a2 - a1*a3)

    b21 = -a3/(a2*a2 - a1*a3)
    b41 = a2/(a2*a2 - a1*a3)

    A = np.matrix(
        [[0, 1, 0, 0],
        [0, 0, a23, 0],
        [0, 0, 0, 1],
        [0, 0, a43, 0]]
    )
    #print(A)

    B = np.matrix(
        [[0],
        [b21],
        [0],
        [b41]]
    )
    #print(B)
    Q = np.diag([1, 10, 20, 1])
    #print(Q)
    R = np.matrix([500])
    #print(R)
    #print(ctrb(A,B))
    #print(np.linalg.matrix_rank(ctrb(A,B)))         # Rank of controllability matrix
    K, S, E = lqr(A, B, Q, R)

    poles = np.array(
        [-2,-3,-4,-5]
    )
    K_poleplacement = acker(A, B, poles)
    #print("K using pole placement is:", K_poleplacement)
    #print("Gain Array:",K)
    D, V = np.linalg.eig(A-B*K)
    #print("The eigen Values are :",D)
    U = -K_poleplacement*X
    return U


#print("X is equal to:\n",X)
