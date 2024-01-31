import numpy as np
from copy import copy

pi = np.pi

def dh(d, theta, a, alpha):
  sth = np.sin(theta)
  cth = np.cos(theta)
  sa  = np.sin(alpha)
  ca  = np.cos(alpha)
  T = np.array([[cth, -ca*sth,  sa*sth, a*cth],
                [sth,  ca*cth, -sa*cth, a*sth],
                [0.0,      sa,      ca,     d],
                [0.0,     0.0,     0.0,   1.0]])
  return T

#def fkine_sci(q):

#  d     = np.array([   q[0] + 7.27,             0,         0])
#  th    = np.array([             0,   q[1] + pi/2,      q[2]])
#  a     = np.array([             0,          0.75,    1.2328])
#  alpha = np.array([          pi/2,          pi/2,         0])

#  T1 = dh(d[0], th[0], a[0], alpha[0])
#  T2 = dh(d[1], th[1], a[1], alpha[1])
# T3 = dh(d[2], th[2], a[2], alpha[2])

#  T = T1.dot(T2).dot(T3)
    
#  return T

def fkine_sci(q): #2:1 Scale

  d     = np.array([   q[0] + 7.345,             0,         0])
  th    = np.array([              0,   q[1] + pi/2,      q[2]])
  a     = np.array([              0,          1.5,        2.5])
  alpha = np.array([           pi/2,          pi/2,         0])

  T1 = dh(d[0], th[0], a[0], alpha[0])
  T2 = dh(d[1], th[1], a[1], alpha[1])
  T3 = dh(d[2], th[2], a[2], alpha[2])

  T = T1.dot(T2).dot(T3)
    
  return T


def jacobian_position(q, delta=0.0001):

  J = np.zeros((3,3))

  T = fkine_sci(q)
  X = T[0:3,3]

  for i in xrange(3):

       dq = copy(q)

       dq[i]= dq[i] + delta

       Tnew = fkine_sci(dq)
       Xnew = Tnew[0:3,3]

       J[0,i] = (Xnew[0] -  X[0])/delta
       J[1,i] = (Xnew[1] -  X[1])/delta
       J[2,i] = (Xnew[2] -  X[2])/delta

  return J

def ikine(xdes, q0):

    epsilon  = 0.001
    max_iter = 1000
    delta    = 0.00001
 
    q = copy(q0)
    J = jacobian_position(q,0.0001)
    
    for i in range(max_iter):
        # Main loop
        f=fkine_sci(q)
        e=xdes-f[0:3,3]  
        q=q+np.dot(np.linalg.pinv(J),e) 

        if(np.linalg.norm(e)<epsilon):
            break
    return q

#def sci_kin(q):
#    fk = np.array([[1.2328 * np.cos(q)], 
#                    [1.2328 * np.sin(q)]])
#    return fk

def sci_kin(q): #2:1 Scale
    fk = np.array([[2.5 * np.cos(q)], 
                    [2.5 * np.sin(q)]])
    return fk

def jacobian_sci_pos(q,delta=0.0001):
    J = np.zeros((2,1))
    
    x = sci_kin(q)
    
    dq = q + delta
    
    dx = sci_kin(dq)
    
    J = (dx - x)/delta
    
    return J