from mpl_toolkits import mplot3d
import numpy as np
import math
import matplotlib.pyplot as plt

l1=80
l2=80
L = 120 # along - x-axis
W = 90 # along - y-axis
Lp=np.array([[100,W/2,-100,1],[100,-W/2,-100,1],[-100,W/2,-100,1],[-100,W/2,-100,1]]) #four leg in world-space
#leg-space
#x=20 #forward
#y=0 
#z=-100 #up
 
#BodyPose - Rotation
roll = math.pi /8# Body xrot
pitch = 0#-math.pi / 4 #math.pi/4# Body YRot
yaw = 0 #math.pi/6 # Body ZRot
#BodyPose - Center of Robot
xm = 0
ym = 0
zm = 0

def setupView(limit):
    ax = plt.axes(projection="3d")
    ax.set_xlim(-limit, limit)
    ax.set_ylim(-limit, limit)
    ax.set_zlim(-limit, limit)
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    return ax

#setupView(110).view_init(elev=20., azim=135)
setupView(200).view_init(elev=12, azim=28)

def legIK(x,y,z):
    # xz-plane
    calf_theta = math.acos((x**2 + z**2 - l1**2 - l2**2) / (2 * l1*l2))
    thigh_theta = math.atan2(z,x) - math.atan2(l2*math.sin(calf_theta), l1 + (l2*math.cos(calf_theta)))
    return(thigh_theta,calf_theta)

def calcLegPoints(angles):

    (theta1,theta2)=angles
    theta1 = -theta1
    theta2 = -theta2
    trans_0 = np.array([0,0,0,1])
    T01 = np.array([[math.cos(theta1), 0, math.sin(theta1), l1*math.cos(theta1)],
                    [0, 1, 0, 0],
                    [-math.sin(theta1), 0, math.cos(theta1), -l1*math.sin(theta1)],
                    [0, 0, 0, 1]
                    ]) 
    trans_1 = np.array([T01[0][3],T01[1][3],T01[2][3],1])
    T12 = np.array([[math.cos(theta2), 0, math.sin(theta2), l2*math.cos(theta2)],
                    [0, 1, 0, 0],
                    [-math.sin(theta2), 0, math.cos(theta2), -l2*math.sin(theta2)],
                    [0, 0, 0, 1]
                    ]) 
    T02 = T01@T12
    trans_2 = np.array([T02[0][3],T02[1][3],T02[2][3],1])
    # T0=np.array([0,0,0,1])
    # T1=T0+np.array([l1*math.cos(theta1),l1*math.sin(theta1),0,0])
    # T2=T1+np.array([l2*math.cos(theta2),l2*math.sin(theta2),0,0])
    #-----
    #T_f = np.array([l2*math.cos(theta1+theta2) + l1 * math.cos(theta1), l2*math.sin(theta1 + theta2) + l1*math.sin(theta1),0,0])
    #print(Point_2)
    #print(np.array([trans_0,trans_1,trans_2]))
    return np.array([trans_0,trans_1,trans_2])

def drawLegPoints(p):
    #สร้างเส้น 3 เส้น L0,L1,L2
    plt.plot([p[0][0],p[1][0],p[2][0]], 
             [p[0][1],p[1][1],p[2][1]],
             [p[0][2],p[1][2],p[2][2]],'k-', lw=3)
    
    plt.plot([p[0][0]],[p[0][1]],[p[0][2]],'ko',lw=2)
    #plt.plot([p[1][0]],[p[1][1]],[p[1][2]],'yo',lw=2)
    plt.plot([p[2][0]],[p[2][1]],[p[2][2]],'ro',lw=2)
   
def bodyIK(roll,pitch,yaw,xm,ym,zm):

    """
    Calculate the four Transformation-Matrices for our Legs
    Rx=X-Axis Rotation Matrix
    Ry=Y-Axis Rotation Matrix
    Rz=Z-Axis Rotation Matrix
    Rxyz=All Axis Rotation Matrix
    T=Translation Matrix
    Tm=Transformation Matrix
    Trb,Trf,Tlb,Tlf=final Matrix for RightBack,RightFront,LeftBack and LeftFront
    """

    Rx = np.array([
        [1, 0, 0, 0], 
        [0, math.cos(roll), -math.sin(roll), 0],
        [0,math.sin(roll),math.cos(roll),0],
        [0,0,0,1]])

    Ry = np.array([
        [math.cos(pitch),0, math.sin(pitch), 0], 
        [0, 1, 0, 0],
        [-math.sin(pitch),0, math.cos(pitch),0],
        [0,0,0,1]])

    Rz = np.array([
        [math.cos(yaw),-math.sin(yaw), 0,0], 
        [math.sin(yaw),math.cos(yaw),0,0],
        [0,0,1,0],
        [0,0,0,1]])

    Rxyz=Rx@Ry@Rz

    T = np.array([[1,0,0,xm],[0,1,0,ym],[0,0,1,zm],[0,0,0,1]])
    Tm = T @ Rxyz
    # Trb = Tm @ np.array([
    #     [math.cos(math.pi/2),0,math.sin(math.pi/2),-L/2],
    #     [0,1,0,-W/2],
    #     [-math.sin(math.pi/2),0,math.cos(math.pi/2),0],
    #     [0,0,0,1]])
    Trb = Tm @ np.array([
        [1,0,0,-L/2],
        [0,1,0,-W/2],
        [0,0,1,0],
        [0,0,0,1]])

    # Trf = Tm @ np.array([
    #     [math.cos(math.pi/2),0,math.sin(math.pi/2),L/2],
    #     [0,1,0,-W/2],
    #     [-math.sin(math.pi/2),0,math.cos(math.pi/2),0],
    #     [0,0,0,1]])
    Trf = Tm @ np.array([
        [1,0,0,L/2],
        [0,1,0,-W/2],
        [0,0,1,0],
        [0,0,0,1]])

    # Tlf = Tm @ np.array([
    #     [math.cos(math.pi/2),0,math.sin(math.pi/2),L/2],
    #     [0,1,0,W/2],
    #     [-math.sin(math.pi/2),0,math.cos(math.pi/2),0],
    #     [0,0,0,1]])
    Tlf = Tm @ np.array([
        [1,0,0,L/2],
        [0,1,0,W/2],
        [0,0,1,0],
        [0,0,0,1]])
    # Tlb = Tm @ np.array([
    #     [math.cos(math.pi/2),0,math.sin(math.pi/2),-L/2],
    #     [0,1,0,W/2],
    #     [-math.sin(math.pi/2),0,math.cos(math.pi/2),0],
    #     [0,0,0,1]])
    Tlb = Tm @ np.array([
        [1,0,0,-L/2],
        [0,1,0,W/2],
        [0,0,1,0],
        [0,0,0,1]])


    return (Tlf,Trf,Tlb,Trb,Tm)

def drawRobot(Lp,bodyIk):

    (Tlf,Trf,Tlb,Trb,Tm)=bodyIk

    FP=[0,0,0,1]
    CP=[x@FP for x in [Tlf,Trf,Tlb,Trb]]

    [plt.plot([x[0]],[x[1]],[x[2]],'mo-') for x in Lp]

    plt.plot([CP[0][0],CP[1][0],CP[3][0],CP[2][0],CP[0][0]],
             [CP[0][1],CP[1][1],CP[3][1],CP[2][1],CP[0][1]],
             [CP[0][2],CP[1][2],CP[3][2],CP[2][2],CP[0][2]], 'bo-', lw=2)

    # Invert local X
    Ix=np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])
    print(Tlf)
    #Q=np.linalg.inv(Tlf)@(Lp[0])
    Q=np.linalg.inv(Tlf)@Lp[0]
    print(Q)
    p=[Tlf@x for x in calcLegPoints(legIK(Q[0],Q[1],Q[2]))]
    drawLegPoints(p)

    Q=np.linalg.inv(Tlb)@Lp[2]
    p=[Tlb@x for x in calcLegPoints(legIK(Q[0],Q[1],Q[2]))]
    drawLegPoints(p)

    #TP=np.array([40,-150,0,1])

    Q=Ix@np.linalg.inv(Trf)@Lp[1]
    p=[Trf@Ix@x for x in calcLegPoints(legIK(Q[0],Q[1],Q[2]))]
    drawLegPoints(p)

    Q=Ix@np.linalg.inv(Trb)@Lp[3]
    p=[Trb@Ix@x for x in calcLegPoints(legIK(Q[0],Q[1],Q[2]))]
    drawLegPoints(p)

#drawLegPoints(calcLegPoints(legIK(x,y,z)))
# for x in range(-100,150,50):
#     drawLegPoints(calcLegPoints(legIK(x,y,z)))
# (Tlf,Trf,Tlb,Trb,Tm)=bodyIK(roll,pitch,yaw,xm,ym,zm)
# FP=[0,0,0,1]
# CP=[x@FP for x in [Tlf,Trf,Tlb,Trb]]
# plt.plot([CP[0][0],CP[1][0],CP[3][0], CP[2][0],CP[0][0]],
#          [CP[0][1],CP[1][1],CP[3][1], CP[2][1],CP[0][1]],
#          [CP[0][2],CP[1][2],CP[3][2], CP[2][2],CP[0][2]], 'bo-', lw=2)
drawRobot(Lp,bodyIK(roll , pitch, yaw ,xm,ym,zm))
plt.get_current_fig_manager().set_window_title('8DOF Quadruped Robot')
plt.show()
print("OK")