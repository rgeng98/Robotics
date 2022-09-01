import math
import numpy as np

# Bob Geng Robotics Library

# Convert Quaternions to Roll Pitch Yaw
def Quat_To_Euler(x, y, z, w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    X = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    Y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    Z = math.atan2(t3, t4)

    return X, Y, Z

#Convert Euler angles to quaternions
def Euler_To_Quat(roll, pitch, yaw):
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    return qx, qy, qz, qw

#Convert spherical coordinates to cartesian
def Spherical_to_Cartesian(p, theta, phi):
    x = p * math.cos(phi) * math.cos(theta)
    y = p * math.cos(phi) * math.sin(theta)
    z = p * math.sin(phi)
    return x, y, z

#convert cartesian coordinats to spherical
def Cartesian_to_Spherical(x, y, z):
    p = math.sqrt(x**2 + y**2 + z**2)
    theta = math.atan2(y, x)
    phi = math.atan2(z,math.sqrt(x**2 + y**2))
    return p, theta, phi

#Calculate the mobility of a planar robot
def Planar_Mobility(N, J1, J2):
    return 3*(N-1) - 2*J1 - 1*J2

# Calculate the mobility of a spatial robot
def Spatial_Mobility(N, J1, J2, J3, J4, J5):
    return 6*(N-1) - 5*J1 - 4*J2 - 3*J3 - 2*J4 - 1*J5

def dot(a,b):
    c = 0
    if len(a) == len(b):
        for i in range(len(a)):
            c = c + a[i] * b[i]
    return float(a[0]*b[0] + a[1]*b[1] + a[2]*b[2])

def Euler_to_Rotation_Matrix(alpha, beta, gamma):
    rot_mat = np.array([[math.cos(alpha)*math.cos(beta), -1*math.sin(alpha)*math.cos(gamma)+math.cos(alpha)*math.sin(beta)*math.sin(gamma), math.sin(alpha)*math.sin(gamma)+math.cos(alpha)*math.sin(beta)*math.cos(gamma)],
                           [math.sin(alpha)*math.cos(beta), math.cos(alpha)*math.cos(gamma)+math.sin(alpha)*math.sin(beta)*math.sin(gamma), -1 * math.cos(alpha)*math.sin(gamma)+math.sin(alpha)*math.sin(beta)*math.cos(gamma)],
                           [-1*math.sin(beta), math.cos(beta)*math.sin(gamma), math.cos(beta)*math.cos(gamma)]])
    return rot_mat

def Rotation_Matrix_to_Euler(r):
    beta = math.atan2(-1*float(r[2][0]), math.sqrt(float(r[0][0])**2+float(r[1][0])**2))
    alpha = math.atan2(float(r[1][0])/math.cos(beta),float(r[0][0])/math.cos(beta))
    gamma = math.atan2(float(r[2][1])/math.cos(beta), float(r[2][2])/math.cos(beta))
    return np.array([alpha, beta, gamma])

def Rotation_Matrix_to_Quaternions(r):
    w = math.sqrt(1+r[0][0] + r[1][1] + r[2][2])/2
    e1 = (r[2][1] - r[1][2])/(4*w)
    e2 = (r[0][2] - r[2][0])/(4*w)
    e3 = (r[1][0] - r[0][1])/(4*w)
    return np.array([e1, e2, e3, w])

def Quaternion_to_Rotation_Matrix(q):
    a11 = 1-2*(q[1]**2+q[2]**2)
    a12 = 2*(q[0]*q[1] - q[2]*q[3])
    a13 =2*(q[0]*q[2] + q[1]*q[3])
    a21 = 2*(q[0]*q[1] + q[2]*q[3])
    a22 = 1-2*(q[0]**2 + q[2]**2)
    a23 = 2*(q[1]*q[2] - q[0]*q[3])
    a31 = 2*(q[0]*q[2] - q[1]*q[3])
    a32 = 2*(q[1]*q[2] + q[0]*q[3])
    a33 = 1-2*(q[0]**2 + q[1]**2)
    R = np.array([[a11, a12, a13],
                      [a21, a22, a23],
                      [a31, a32, a33]])
    return R

def Matrix_Inverse(a):
    a = np.array(a)
    return np.linalg.inv(a)

def rotation_3d(a, b):
    xa = np.array([a[0][0],a[0][1],a[0][2]])
    ya = np.array([a[1][0],a[1][1],a[1][2]])
    za = np.array([a[2][0],a[2][1],a[2][2]])

    xb = np.array([b[0][0],b[0][1],b[0][2]])
    yb = np.array([b[1][0],b[1][1],b[1][2]])
    zb = np.array([b[2][0],b[2][1],b[2][2]])

    rotation_matrix = np.array([[dot(xa, xb), dot(ya, xb), dot(za, xb)],
                                    [dot(xa, yb), dot(ya, yb), dot(za, yb)],
                                    [dot(xa, zb), dot(ya, zb), dot(za, zb)]])
    return np.array(rotation_matrix)

def plot3ax(x,y,z, xnn,ynn,znn):
    fig = plt.figure(1)
    plt.clf()
    ax = plt.axes(projection='3d')
    ax.set_xlim3d(-10, 10)
    ax.set_ylim3d(-10, 10)
    ax.set_zlim3d(-10, 10)
    plt.title('Robot Animation')
    ax.plot3D(x,y,z,color='gray')
    ax.scatter(xnn,ynn,znn,color='green')

def invers_quaternion_axis_angle(q):
    theta = 2*math.acos(q[3])
    kx = q[0]/math.sin(theta/2)
    ky = q[1]/math.sin(theta/2)
    kz = q[2]/math.sin(theta/2)
    return np.array([kx, ky, kz, theta])

def axis_angle_to_quaternion(a):
    q1 = a[0] * math.sin(a[3]/2)
    q2 = a[1] * math.sin(a[3]/2)
    q3 = a[2] * math.sin(a[3]/2)
    q4 = math.cos(a[3]/2)
    return np.array([q1, q2, q3, q4])

def Transformation_matrix_rp(e,p):
    r = Euler_to_Rotation_Matrix(e[0], e[1], e[2])
    t = np.array([[r[0][0], r[0][1], r[0][2], p[0][0]],
                      [r[1][0], r[1][1], r[1][2], p[1][0]],
                      [r[2][0], r[2][1], r[2][2], p[2][0]],
                      [      0,       0,       0,      1]])
    return t

def Inverse_Transformation(T):
    R = np.array([[T[0][0], T[0][1], T[0][2]],
                  [T[1][0], T[1][1], T[1][2]],
                  [T[2][0], T[2][1], T[2][2]]])
    P = np.array([[T[0][3]],[T[1][3]], [T[2][3]]])
    r_t = np.transpose(R)
    p_t = -1 * np.matmul(np.linalg.inv(R), P)
    T = np.array([[r_t[0][0], r_t[0][1], r_t[0][2], p_t[0][0]],
                 [r_t[1][0], r_t[1][1], r_t[1][2], p_t[1][0]],
                 [r_t[2][0], r_t[2][1], r_t[2][2], p_t[2][0]],
                 [0, 0, 0, 1]])
    return T

def Point_Transform(T, P):
    a = np.array([[P[0][0]],[P[1][0]],[P[1][0]],[1]])
    return np.matmul(T,a)

def SCARA_DH_parameters(th1, th2,th4, d):
    dh = np.array([[0,    0,     0, th1],
                   [0,    0.300, 0, th2],
                   [np.pi,0.250, d, 0],
                   [0,    0,     0, th4]])
    return dh

def DH_to_Translations(DH):
    #Parameters are to be input as [alpha_i-1 a_i-1 d_i theta_i]
    #Initialize the solution as the identity matrix for multiplication in a Loop
    tr = np.array[[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]]
    #Loop through the rows on the DH parameter table
    for parameters in DH:
        #Calculate DH parameters for each row of DH parameters
        t = np.array([[np.cos(parameters[3]), -np.sin[3], 0, parameters[1]],
                      [np.sin(parameters[3])*np.cos(parameters[0]), np.cos(parameters[3])*np.sin(parameters[0]),-np.sin(parameters[0]), -1*parameters[2]*np.sin(parameters[0])],
                      [np.sin(parameters[3])*np.sin(parameters[0]),np.cos(parameters[3])*np.sin(parameters[0]),np.cos(parameters[0]), -1*parameters[2],*np.cos(parameters[0])],
                      [0, 0, 0, 1]])
        #Multiply solution by each transformation matrix derived from DH parameters
        tr = np.matmul(tr,t)
    return tr

