#!/usr/bin/env python
import rospy
from matplotlib import pyplot as plt
from matplotlib import animation as animation
from sensor_msgs.msg import LaserScan
from math import cos, sin
import math

ranges = [0]*720
angles = []
for i in range(720):
    angles.append(float(i)*math.pi/360.0)

fig = plt.figure()
ax = fig.add_subplot(1,1,1)

Xi = []
Yi = []
Xo = []
Yo = []

def standardAngle(theta):
    """chuan hoa goc ve vung [0->2*pi]
    """
    tmp = theta
    while (tmp < 0):
        tmp += 2*math.pi
    while (tmp >= 2*math.pi):
        tmp -= 2*math.pi
    return tmp

def pointFromDecart2Polar(P):
    """Chuyen toa do he Decart (x,y) sang he Polar (r,theta)
    """

    r = math.sqrt(P[0]*P[0] + P[1]*P[1])
    if P[0] == 0:
        if(P[1] > 0):
            theta = math.pi/2
        else:
            theta = - math.pi/2
    else:
        theta = math.atan(P[1]/P[0])
        if P[0] < 0:
            theta += math.pi

    theta = standardAngle(theta)
    return [r,theta]

def lineFromPoints(P, Q):
    """Tim phuong trinh duong thang di qua 2 diem P,Q
    Args:
        P ([x,y]): First point
        Q ([x,y]): Second point
    Return:
        [a,b,c]: ax + by + c = 0
    """
    a = Q[1] - P[1]
    b = P[0] - Q[0]
    c = -(a*P[0] + b*P[1])
    return [a,b,c]

def distanceFromOxy2Line(L):
    """Tinh khoang cach tu goc toa do Oxy(0,0) toi duong thang L

    Args:
        L ([a,b,c]): ax + by + c = 0

    Returns:
        float: Khoang cach
    """
    return abs(float(L[2])/math.sqrt(L[0]**2 + L[1]**2))

def closestPointOxy2Line(L):
    """Tim Diem gan voi goc O(0,0) nhat tren duong thang L[a,b,c]

    Args:
        L ([a,b,c]): Duong thang ax + by + c = 0

    Returns:
        [x,y]: Hinh chieu cua O(0,0) len L
    """
    x = (-L[0]*L[2]) / (L[0]**2 + L[1]**2)
    y = (-L[1]*L[2]) / (L[0]**2 + L[1]**2)
    return [x, y]

def zoneFromTwoPoints(P, Q):
    """Tim duong thang trong he Polar di qua 2 diem
        r = p/(cos(theta - phi))

    Args:
        P ([x, y]): The first point
        Q ([x, y]): The second point

    Returns:
        [p, phi]: Hai he so cua duong thang trong he toa do Polar
    """
    L = lineFromPoints(P, Q)
    p = distanceFromOxy2Line(L)
    H = closestPointOxy2Line(L)
    H_polar = pointFromDecart2Polar(H)

    return p, H_polar[1]

def isPointInZone(C, P, Q):
    """Xet xem diem C co nam trong vung duoc tao boi Oxy va P, Q hay khong

    Args:
        C ([r, theta]): Diem can xet (Polar)
        P ([x, y]): Diem gioi han 1 (Decart)
        Q ([x, y]): Diem gioi han 2 (Decart)

    Returns:
        bool:   True - C thuoc trong vung OPQ
                False - C khong thuoc vung OPG
    """
    p, phi = zoneFromTwoPoints(P, Q)
    theta_P = pointFromDecart2Polar(P)[1]
    theta_Q = pointFromDecart2Polar(Q)[1]
    print("p: %.2f - phi: %.2f"%(p, phi))
    print("theta_P: %.2f - theta_Q: %.2f"%(theta_P, theta_Q))

    r = p/math.cos(C[1] - phi)
    s_phi = (phi - theta_P)*(phi - theta_Q)
    s_theta = (C[1] - theta_P)*(C[1] - theta_Q)
    if s_phi * s_theta >= 0 and C[0] <= r:
        return True
    else:
        return False

sliceArr = []
MaxSlice = 5
MinObs = 3
obs = False

def scanCallBack(msg):
    global Xi, Yi, Xo, Yo, sliceArr, obs
    obs = False
    ranges = list(msg.ranges)
    Xi = []
    Yi = []
    Xo = []
    Yo = []
    for range, angle in zip(ranges, angles):
        c_in = False
        for z in zonesInPolar:
            r = z[0]/math.cos(angle-z[1])
            s_theta = (angle - z[2])*(angle - z[3])*z[4]
            if(s_theta <= 0) and range <= r:
                c_in = True
                # print("{} in zone {}".format(c, z[5]))
                # print("p: %.2f - phi: %.2f"%(z[0], z[1]))
                # print("theta_P: %.2f - theta_Q: %.2f"%(z[2], z[3]))
                break
        if c_in:
            Xi.append(range*cos(angle))
            Yi.append(range*sin(angle))
            sliceArr.append(1)
        else:
            Xo.append(range*cos(angle))
            Yo.append(range*sin(angle))
            sliceArr.append(0)

        if len(sliceArr) > MaxSlice:
            sliceArr = sliceArr[-MaxSlice:]
        if sum(sliceArr) > MinObs:
            obs = True
            # print('Have Obstacle')

def animate(i, xi, yi, xo, yo, vx, vy):
    # draw here
    # global Xi, Yi, Xo, Yo
    xi = Xi
    yi = Yi
    xo = Xo
    yo = Yo
    vx = Vx
    vy = Vy

    ax.clear()
    if obs:
        ax.fill_between(vx, vy, facecolor='y')
    else:
        ax.fill_between(vx, vy, facecolor='g')

    ax.scatter(xi, yi, color='r', s=0.1)
    ax.grid(True)
    ax.scatter(xo, yo, color='b', s=0.1)
    ax.scatter(0,0,c='black')

    ax.axis('equal')
    ax.set_xlim([-3,3])
    ax.set_ylim([-3,3])

if __name__ == "__main__":
    rospy.loginfo("init node show_scan")
    rospy.init_node("show_scan")

    Vetexs = [[0, -0.5], [0.5, -0.5], [0.5, 0.5], [-2.5, 0.5], [-2.5, 0],[0, -0.5]]
    zonesInPolar = []

    P = Vetexs[0]
    for i in range(1, len(Vetexs)):
        Q = Vetexs[i]
        p, phi = zoneFromTwoPoints(P, Q)
        theta_P = pointFromDecart2Polar(P)[1]
        theta_Q = pointFromDecart2Polar(Q)[1]
        if(abs(theta_Q - theta_P) < math.pi):
            k = 1
        else:
            k = -1
        zonesInPolar.append([p, phi, theta_P, theta_Q, k, i])
        P = Vetexs[i]

    Vx = [0]
    Vy = [0]
    for v in Vetexs:
        Vx.append(v[0])
        Vy.append(v[1])
    Vx.append(0)
    Vy.append(0)

    rospy.Subscriber("/scan", LaserScan, scanCallBack)
    ani = animation.FuncAnimation(fig, animate, fargs=(Xi, Yi, Xo, Yo, Vx, Vy), interval=100)

    plt.show()
    # while not rospy.is_shutdown():
        # rospy.sleep(1)



