from cgi import test
from turtle import Vec2D
import numpy as np
from scipy.optimize import fsolve
#from gekko 

def f(x,n):
    y=np.zeros(np.size(x))
    y[0] = n[0,0]*x[0] + n[0,1]*x[1] + n[0,2]*x[2]- n[0,3]
    y[1] = n[1,0]*x[0] + n[1,1]*x[1] + n[1,2]*x[2]- n[1,3]
    y[2] = n[2,0]*x[0] + n[2,1]*x[1] + n[2,2]*x[2]- n[2,3]
    return y

C = np.array([[1, -1, 3, 5],[1, 1 ,6, 12],[3, -2, 2,10]])
x0 = np.array([1,1,1])
x  = fsolve(f,x0,C)
print(x)

A = np.array([[1, -1, 3],[1, 1 ,6],[3, -2, 2]])
B = np.array([5, 12 , 10])
Y = np.linalg.solve(A,B)
print(Y)


# Ps = [-238.8, -660.1, 489.2]
# V1 = [-3.7, 11.5, -65.7]
# V2 = [8.2, 17.7, -65.8]
# V4 = [23.3, 4.4, -65.1]

