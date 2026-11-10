#!/usr/bin/env python
import math

def standard_angle(theta):
    """chuan hoa goc ve vung [0->2*pi]
    """
    tmp = theta
    while (tmp < 0):
        tmp += 2*math.pi
    while (tmp >= 2*math.pi):
        tmp -= 2*math.pi
    return tmp

def point_descart_to_polar(P):
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

    theta = standard_angle(theta)
    return [r,theta]

def line_from_two_points(P, Q):
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

def distance_from_Oxy_to_line(L):
    """Tinh khoang cach tu goc toa do Oxy(0,0) toi duong thang L

    Args:
        L ([a,b,c]): ax + by + c = 0

    Returns:
        float: Khoang cach
    """
    return abs(float(L[2])/math.sqrt(L[0]**2 + L[1]**2))

def closest_point_form_Oxy_to_line(L):
    """Tim Diem gan voi goc O(0,0) nhat tren duong thang L[a,b,c]

    Args:
        L ([a,b,c]): Duong thang ax + by + c = 0

    Returns:
        [x,y]: Hinh chieu cua O(0,0) len L
    """
    x = (-L[0]*L[2]) / (L[0]**2 + L[1]**2)
    y = (-L[1]*L[2]) / (L[0]**2 + L[1]**2)
    return [x, y]

def zone_from_two_points(P, Q):
    """Tim duong thang trong he Polar di qua 2 diem
        r = p/(cos(theta - phi))

    Args:
        P ([x, y]): The first point
        Q ([x, y]): The second point

    Returns:
        [p, phi]: Hai he so cua duong thang trong he toa do Polar
    """
    L = line_from_two_points(P, Q)
    p = distance_from_Oxy_to_line(L)
    H = closest_point_form_Oxy_to_line(L)
    H_polar = point_descart_to_polar(H)

    return p, H_polar[1]

def is_point_in_zone(C, P, Q):
    """Xet xem diem C co nam trong vung duoc tao boi Oxy va P, Q hay khong

    Args:
        C ([r, theta]): Diem can xet (Polar)
        P ([x, y]): Diem gioi han 1 (Decart)
        Q ([x, y]): Diem gioi han 2 (Decart)

    Returns:
        bool:   True - C thuoc trong vung OPQ
                False - C khong thuoc vung OPG
    """
    p, phi = zone_from_two_points(P, Q)
    theta_P = point_descart_to_polar(P)[1]
    theta_Q = point_descart_to_polar(Q)[1]
    print("p: %.2f - phi: %.2f"%(p, phi))
    print("theta_P: %.2f - theta_Q: %.2f"%(theta_P, theta_Q))

    r = p/math.cos(C[1] - phi)
    s_phi = (phi - theta_P)*(phi - theta_Q)
    s_theta = (C[1] - theta_P)*(C[1] - theta_Q)
    if s_phi * s_theta >= 0 and C[0] <= r:
        return True
    else:
        return False

def get_point_from_string(str):
    tmp = str[2:-2]
    while (' ' in tmp):
        tmp = tmp.replace(' ', '')
    tmp1 = tmp.split('],[')
    X = [0]
    Y = [0]
    for p_str in tmp1:
        [x, y] = p_str.split(',')
        X.append(float(x))
        Y.append(float(y))
    X.append(0)
    Y.append(0)

def str2arr(arr_str):
    """ Convert tu arr dang string: '[[],[],[]...]' sang list: [[],[],[]...]
    """
    tmp = arr_str[2:-2]
    while ' ' in tmp:
        tmp = tmp.replace(' ', '')
    tmp_arr = tmp.split('],[')
    vertexes = []
    for p in tmp_arr:
        [x,y] = p.split(',')
        vertexes.append([float(x), float(y)])
    return vertexes

if __name__ == '__main__':
    field_1 = '[[1.0, 0.5], [-0.7, 0.5], [-0.7, -0.5], [1.0, -0.5], [1.0, 0.5]]'
    get_point_from_string(field_1)
    arr = str2arr(field_1)
    print(arr)
    print(type(arr))