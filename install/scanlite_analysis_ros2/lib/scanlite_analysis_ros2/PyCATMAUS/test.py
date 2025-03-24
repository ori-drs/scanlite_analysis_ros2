from sympy.solvers import solve
import numpy as np
import sympy as sym

D1 = 7.2
D2 = 17.8
D3 = 14.0

k1 = sym.Symbol('k1')
k2 = sym.Symbol('k2')
k3 = sym.Symbol('k3')

v1 = np.array([-3.7, 11.5, -65.6])
v2 = np.array([8.2, 17.7, -65.8])
v3 = np.array([23.3, 4.4, -65.1])

# eqn1 = sym.Eq(sum((k1*v1-k2*v2)**2),D1**2)
# eqn2 = sym.Eq(sum((k1*v1-k3*v3)**2),D2**2)
# eqn3 = sym.Eq(sum((k2*v2-k3*v3)**2),D3**2)
# sol = sym.solve([eqn1,eqn2,eqn3],[k1,k2,k3])
# print (sol)

n = np.array([[1, -1, 3, 5],[1, 1 ,6, 12],[3, -2, 2,10]])
y0 = n[0,0]*k1 + n[0,1]*k2 + n[0,2]*k3- n[0,3]
y1 = n[1,0]*k1 + n[1,1]*k2 + n[1,2]*k3- n[1,3]
y2 = n[2,0]*k1 + n[2,1]*k2 + n[2,2]*k3- n[2,3]
sol2 = sym.solve([y0,y1,y2],[k1,k2,k3])
print (sol2)