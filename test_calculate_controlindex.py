import math
import numpy as np

####  by default the vechicle num 1 get information from virtual leader
Robot = 'Robot2'
#### global nRob
nRob = 2

# adjacent matrix of comunication topology, also defined in 'topologie.cfg'
Topologie = np.mat([[1,1,1,1,1,1],
                    [1,1,1,1,1,1],
                    [1,1,1,1,1,1],
                    [1,1,1,1,1,1],
                    [1,0,0,0,0,1],
                    [0,0,1,0,0,0]])


#### position initialisition
Theta = 0
X = 0
Y = 0
Theta_last = 0
X_last = 0
Y_last = -0.9

#### control parameter initialision
beta = 0.01
gamma = 0.06
k0 = 2
b = 1
P=5
I=10
D=1
e1_R = 0
e2_R = 0
e3_R = 0
e1_L = 0
e2_L = 0
e3_L = 0

#### the 4 important control index
u1 = 0
u2 = 0
z1 = 0
z2 = 0
z3 = 0

#### mechanical parameter initialision
r = 0.0216  #### radius for wheel
L = 0.087   #### distance for two wheel

#### geometrical formation parameter
Px = 0
Py = 0.2

#### virtual leader information ---(it's only used for vheicle num 1)
# time = 0
z20 = k0
u10 = 0.1

#### pre-filter parameter
N = 1
T1 = 0
T2 = 0
T1_last = 0
T2_last = 0
T1_M_last = 0
T2_M_last = 0
Threshold = 150
T1_invalid = 0
T2_invalid = 0
Lowpass = 0.5

####-------------- info class --------------###############
class Info:
    def __init__(self, from_num, z1, z2):
      self.from_num = from_num
      self.z1 = z1
      self.z2 = z2

    def print_info(self):
        print 'get info'+'z1='+self.z1+'z2='+self.z2+'message from' + self.from_num

def calculate_vitrualleader(t):
#### calculate vitrual leader position information
    print 'calculate_virtualleader'
    global z20
    w = 0.157079
    x = 3-3*math.cos(w*t)
    y = 3*math.sin(w*t)

    z10 = 0.1*t
    z20 = z20
    return z10,z20

def calculate_controlindex():
#### use local info list to calculate control law index U1 U2
    print 'calculate_controlindex'
    global z1,z2,z3,u1,u2,Theta,X,Y,Px,Py
    sum1 = 0
    sum2 = 0
    dz20 = 0
    z1 = Theta
    z3 = (X-Px)*math.sin(Theta)-(Y-Py)*math.cos(Theta)
    z2 = (X-Px)*math.cos(Theta)+(Y-Py)*math.sin(Theta)+k0*z3

    for i in list_info:
        if(i==0):
            continue
        sum1 += Topologie[nRob-1,i-1]*pow((z1-list_info[i].z1),Topologie[nRob-1,i-1])
    for i in list_info:
        if(i==0):
            continue
        sum2 += Topologie[nRob-1,i-1]*pow((z2-list_info[i].z2),Topologie[nRob-1,i-1])
    u1 = u10 -beta*sum1 - gamma*b*(z1-list_info[0].z1)
    u2 = dz20 - beta*sum2 - gamma*b*(z2-list_info[0].z2)-k0*math.fabs(u1)*z2    ##### need to know how dz20
    return u1,u2

def calculate_vehiclestate(u1,u2):
#### use U1,U2,two index to caculate vehicle stat
    print('calculate_vechiclestate')
    Angular_Speed= u1
    Line_Speed = u2+(1+math.pow(k0,2))*u1*z3
    w = Angular_Speed
    v = Line_Speed
    return w,v

def calculate_wheelspeed(w,v):
#### use w,v,two index to caculate vehicle wheel stat
    print('calculate_wheelspeed')
    Rotation_Speed_L = (v-w*L/2)/r
    Rotation_Speed_R = (v+w*L/2)/r
    w1 = Rotation_Speed_L
    w2 = Rotation_Speed_R
    return w1,w2
####--------------- local info list -----------##############


list_info = {}
#info1 = Info(0,10,10)
#info2 = Info(2,0,2.0)
#list_info[0]=info1
t =0
for i in range(1,100):
    z10,z20 = calculate_vitrualleader(t)
    list_info[0] = Info(0,z10,z20)
    u1,u2 = calculate_controlindex()
    w,v = calculate_vehiclestate(u1,u2)
    w1,w2=calculate_wheelspeed(w,v)
    duty1 = w1/8.8*180/math.pi
    duty2 = w2/8.8*180/math.pi
    t += 0.1