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
    GC = z
    """
    OG = math.sqrt(x**2 + y**2)
    HG = math.sqrt(x**2+y**2-l0**2)
    HG_offset = HG - hip_offset
    HC = math.sqrt(HG_offset**2 + z**2)
    #hip_theta = math.pi - math.atan2(y,x) - math.atan2(HG,OG)
    hip_theta = - math.atan2(y,x) - math.atan2(HG,-l0)
    
    D = (HC**2 - l1**2 - l2**2) / (2*l1*l2)
    alpha = math.acos(D)
    #calf_theta = math.pi - alpha
    calf_theta = alpha

    thigh_theta = math.atan2(z,HG_offset) - math.atan2(l2*math.sin(calf_theta),l1+l2*math.cos(calf_theta))

    print("12DOFLeg",hip_theta, thigh_theta, calf_theta)
    return(hip_theta,thigh_theta,calf_theta)

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

#legIK(x,y,z) 
drawLegPoints(calcLegPoints(legIK(x,y,z)))
#drawLegPoints(calcLegPoints(exampleLegIK(x,y,z)))

print("OK")