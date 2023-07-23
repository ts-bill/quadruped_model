from mpl_toolkits import mplot3d
import numpy as np
#from tkinter import *
import math
import matplotlib.pyplot as plt
#link legth
l0=25
hip_offset=20
l1=80
l2=80


#target point
x=-55
y=-100 #up
z=20 #forward


def setupView(limit):
    ax = plt.axes(projection="3d")
    ax.set_xlim(-limit, limit)
    ax.set_ylim(-limit, limit)
    ax.set_zlim(-limit, limit)
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    return ax
setupView(110).view_init(elev=20., azim=135)

def legIK(x,y,z):

    """
    x/y/z=Position of the Foot in Leg-Space

    h1
    """
    h1 = math.sqrt(l0**2 + hip_offset**2)
    h2 = math.sqrt(z**2 + y**2)

    alpha_0 = math.atan2(y,z)
    alpha_1 = math.atan2(hip_offset,l0)
    alpha_2 = math.atan2(l0,hip_offset)
    alpha_3 = math.asin((h1*math.sin(alpha_2+math.radians(90))) / h2)
    alpha_4 = math.radians(90) - (alpha_3 + alpha_2)
    print(alpha_0)
    alpha_5 = alpha_1 - alpha_4

    hip_theta = alpha_0 - alpha_5
    r_0 = (h1*math.sin(alpha_4)) / math.sin(alpha_3)
    H = math.sqrt(r_0**2 + x**2)
    delta = math.asin(x/H)
    thigh_theta = math.acos((H**2 + l1**2 - l2**2) / (2*H*l1)) - delta
    calf_theta = math.acos((l2**2 + l1**2 - H**2) / (2 * l2 * l1)) 
    print("12DOFLeg",hip_theta, thigh_theta, calf_theta)

    return(hip_theta,thigh_theta,calf_theta)
def exampleLegIK(x,y,z):

    """
    x/y/z=Position of the Foot in Leg-Space

    F=Length of shoulder-point to target-point on x/y only
    G=length we need to reach to the point on x/y
    H=3-Dimensional length we need to reach
    """

    F=math.sqrt(x**2+y**2-l0**2)
    G=F-hip_offset  
    H=math.sqrt(G**2+z**2)

    theta1=-math.atan2(y,x)-math.atan2(F,-l0)

    D=(H**2-l1**2-l2**2)/(2*l1*l2)
    theta3=math.acos(D) #calf_theta

    theta2=math.atan2(z,G)-math.atan2(l2*math.sin(theta3),l1+l2*math.cos(theta3)) #thigh_theta

    print(theta1, theta2, theta3)
    return(theta1,theta2,theta3)

def calcLegPoints(angles): # put ceta to forward kinematics

    (hip_theta,thigh_theta,calf_theta)=angles
    theta23=thigh_theta+calf_theta

    T0=np.array([0,0,0,1])
    T1=T0+np.array([-l0*math.cos(hip_theta),l0*math.sin(hip_theta),0,0])
    T2=T1+np.array([-hip_offset*math.sin(hip_theta),-hip_offset*math.cos(hip_theta),0,0])
    T3=T2+np.array([-l1*math.sin(hip_theta)*math.cos(thigh_theta),-l1*math.cos(hip_theta)*math.cos(thigh_theta),l1*math.sin(thigh_theta),0])
    T4=T3+np.array([-l2*math.sin(hip_theta)*math.cos(theta23),-l2*math.cos(hip_theta)*math.cos(theta23),l2*math.sin(theta23),0])
    
    return np.array([T0,T1,T2,T3,T4])

def drawLegPoints(p):
    
    #สร้างเส้น 3 เส้น L0,L1,L2
    plt.plot([p[0][0],p[1][0],p[2][0],p[3][0],p[4][0]], 
             [p[0][1],p[1][1],p[2][1],p[3][1],p[4][1]],
             [p[0][2],p[1][2],p[2][2],p[3][2],p[4][2]], 'k-', lw=3)
    #สร้างจุดตรง shoulder
    plt.plot([p[0][0]],[p[0][1]],[p[0][2]],'bo',lw=2)
    #สร้างจุดตรง 
    plt.plot([p[1][0]],[p[1][1]],[p[1][2]],'go',lw=2)
    #สร้างจุดตรง 
    plt.plot([p[2][0]],[p[2][1]],[p[2][2]],'bo',lw=2)
    #สร้างจุดตรง 
    plt.plot([p[3][0]],[p[3][1]],[p[3][2]],'bo',lw=2)
    #สร้างจุดตรง contract point
    plt.plot([p[4][0]],[p[4][1]],[p[4][2]],'ro',lw=2)
    plt.get_current_fig_manager().set_window_title('12DOF Quadruped Robot')
    plt.show()
  
drawLegPoints(calcLegPoints(legIK(x,y,z)))
#drawLegPoints(calcLegPoints(exampleLegIK(x,y,z)))

print("OK")