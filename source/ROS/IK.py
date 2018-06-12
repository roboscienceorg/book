#!/usr/bin/env python
import rclpy
from std_msgs.msg import Float32
from std_msgs.msg import Int8
import math


def callbackX(data):
    global t1, t2
    t1 = data.data

def callbackY(data):
    global t1, t2
    t2 = data.data
    convert(t1,t2)


def convert(t1,t2):
     global pub, a1, a2, u, v
     x = a1*math.cos(t1) + a2*math.cos(t1+t2)
     y = a1*math.sin(t1) + a2*math.sin(t1+t2)
     print (x, y)

global a1, a2, flag, i, t1, t2
rclpy.init(args=None)
node = rclpy.create_node('InverseKcheck')
subx = node.create_subscription(Float32, 'ConfigspacePathX', callbackX)
suby = node.create_subscription(Float32, 'CondfigspacePathY', callbackY)

#Initialize global variables
a1, a2 = 10.0, 10.0
flag = 0
i = 0
t1, t2 = 0.0, 0.0

while rclpy.ok():
   rclpy.spin_once(node)
