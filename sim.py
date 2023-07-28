import numpy as np
import matplotlib.pyplot as plt
import time
from math import *

a1 = 15
a2 = 10
step = 0.1

#Setup Arrays
t = np.arange(0, 2*np.pi+step, step)
x = 5*np.cos(t) + 10
y = 5*np.sin(t) + 8

#Compute joint angles and check them
a1 = 15.0
a2 = 10.0
d = (x*x + y*y - a1*a1 - a2*a2)/(2*a1*a2)
t2 = np.arctan2(-np.sqrt(1.0-d*d),d)
t1 = np.arctan2(y,x) - np.arctan2(a2*np.sin(t2),a1
                +a2*np.cos(t2))
xsim1 = a1*np.cos(t1)
ysim1 = a1*np.sin(t1)
xsim = a2*np.cos(t1+t2) + xsim1
ysim = a2*np.sin(t1+t2) + ysim1

plt.figure(1)
plt.subplot(131)
plt.xlim(0, 20)
plt.ylim(0, 15)
plt.ylabel('Y')
plt.title('Requested Path')
axes = plt.gca()
axes.set_aspect('equal')
plt.plot(x,y)

plt.subplot(132)
plt.xlim(0.75,2.0)
plt.ylim(-2.75,-1.5)
plt.xlabel('Theta1')
plt.ylabel('Theta2')
plt.title('Joint Angles for Path')
axes = plt.gca()
axes.set_aspect('equal')
plt.plot(t1,t2)

plt.subplot(133)
plt.xlim(0, 20)
plt.ylim(0, 15)
plt.xlabel('X')
plt.ylabel('Y')
plt.title('Traversed Path')
axes = plt.gca()
axes.set_aspect('equal')
plt.plot(xsim,ysim)

fig = plt.gcf()
fig.tight_layout(h_pad = 4)
fig.set_size_inches(8,3.5)
fig.suptitle('Output for circle trace')

plt.savefig("twolinkexample.svg")
plt.savefig("twolinkexample.pdf")
plt.show()

