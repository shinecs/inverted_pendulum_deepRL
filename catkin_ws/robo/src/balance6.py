#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
import numpy as np
from modelling_poleplacement import *

X = np.matrix([[0.0],
              [0.0],
              [0.0],
              [0.0]]
)
delta = 0.0
alpha_range = 0

def callback(data):
    global X
    joint1 = data.position[0]
    joint2 = data.position[1]
    joint1_velocity = data.velocity[0]
    joint2_velocity = data.velocity[1]
    X = np.matrix(
    [[joint1],
     [joint1_velocity],
     [joint2],
     [joint2_velocity]]
    )
    for i in [0,2]:
        if(X[i] > np.pi):
            while(X[i] > np.pi):
                X[i] = X[i] - 2*np.pi
        elif(X[i] < -np.pi):
            while(X[i] < -np.pi):
                X[i] = X[i] + 2*np.pi
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", joint2)

def callback2(data):
    global delta
    delta = data.data

def is_alpha_in_range(alpha_list):
    for x in alpha_list:
        if(abs(x) > alpha_range):
            return False
    return True

def main():
    global alpha_range
    pub = rospy.Publisher('/robo/joint1_position_controller/command', Float64, queue_size=10)
    pub_setvalue = rospy.Publisher('/setpoint', Float64, queue_size=10)
    pub_state = rospy.Publisher('/state', Float64, queue_size=10)
    pub_rqt_joint2 = rospy.Publisher('/joint1_visualize', Float64, queue_size=10)
    rospy.init_node('joint1_control_node', anonymous=True)
    rospy.Subscriber("/robo/joint_states", JointState, callback)
    rospy.Subscriber("/control_effort", Float64, callback2)
    
    rate = rospy.Rate(1000) # 1000hz
    s = 0
    alpha_list = []
    alpha_range = 0.003

    while not rospy.is_shutdown():
        #rospy.loginfo("Joint angle: %s"%joint2)
        pub_rqt_joint2.publish(X[2])
        if(len(alpha_list) >= 10):
            alpha_list.pop(0)
        alpha_list.append(float(X[2]))
        if(not is_alpha_in_range(alpha_list)):
            s = 1
            if(abs(float(X[2])) < 1.0):
                pub_setvalue.publish(0.0)
                pub_state.publish(float(X[2]))
                effort = 1.0 * delta
            else:
                effort = 0.0
        else:
            s = 0
            effort = 1.0 * float(compute_input(X))
        pub.publish(effort)
        #rospy.loginfo("s: %s ,Effort: %s , Alpha: %s"%(s,effort,float(X[2]),))
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
