#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import sqrt, atan2, exp, atan, cos, sin, acos, pi, asin, atan2
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from time import sleep
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs.msg import LaserScan
import tf
import sys
import numpy as np

global Usat
Usat = 5.0
# Frequencia de simulacao no stage
global freq
freq = 100  # Hz

# Estados do robo
global x_n, y_n, theta_n
x_n = 0.0  # posicao x atual do robo
y_n = 0.0  # posicao y atual do robo
theta_n = 0.0  # orientacao atual do robo

# Estados do robo
global x_goal, y_goal
x_goal = 0
y_goal = 0

# Relativo ao feedback linearization
global d
d = 0.80

# Ganho atrativo e repulsivo
global Kp
Kp = 1

global v_fw, w_z
v_fw = 0.0
w_z = 0.0

global goToGoal
goToGoal = False

global L
L = 2.0

global state
state = 'MOVE_TO_GOAL'

global max_range
max_range = 5.0

global min_heuristic
min_heuristic = 10000.0

# Rotina callback para a obtencao da pose do robo
def callback_pose(data):
    global x_n, y_n, theta_n
    x_n = data.pose.pose.position.x  # posicao 'x' do robo no mundo
    y_n = data.pose.pose.position.y  # posicao 'y' do robo no mundo
    x_q = data.pose.pose.orientation.x
    y_q = data.pose.pose.orientation.y
    z_q = data.pose.pose.orientation.z
    w_q = data.pose.pose.orientation.w
    euler = euler_from_quaternion([x_q, y_q, z_q, w_q])
    theta_n = euler[2]  # orientaco 'theta' do robo no mundo
    return

def calcDistance(x_n, y_n, x_d, y_d):
    return sqrt(((x_d - x_n)**2 + (y_d - y_n)**2))

def calcAttractiveForce(x_goal, y_goal):
    Ka = 1.0
    dx = x_goal - x_n
    dy = y_goal - y_n
    rho = np.array([dx,dy])
    norm_rho = np.sqrt(dx**2 + dy**2)
    if(norm_rho <= 0.5):
        attr_force = Ka*rho
    else:
        attr_force = Ka*rho/norm_rho
    return attr_force

def isGoalFree(laserData):
    global x_goal, y_goal, x_n, y_n
    goal_angle = atan2(y_goal - y_n, x_goal - x_n)
    ratio=2*pi/1000
    j=goal_angle/ratio
    position = 499 + int(np.ceil(j))
    if (laserData.ranges[position] == max_range):
    	print("position", position)
    	return True
    else:
    	return False

def calcRepulsiveForces(laserData):
    global Kr, min_d
    Kr = 10000.0
    min_d = 4.0
    #print(laserData)
    rep_force = np.array([0.0,0.0])
    for dist, theta, x, y, d in laserData:
        if dist <= min_d:
            fx = Kr*(2*x*(min_d - dist)/(min_d*dist**4))
            fy = Kr*(2*y*(min_d - dist)/(min_d*dist**4))
            #fx = Kr*(1/(range**2))
            #fy = Kr*(1/(range**2))
            #print("fx",fx, "fy ",fy)
            #rep_force[0] += 
            #rep_force[1] += rep_force[1]+fy
            rep_force[0] = fx
            rep_force[1] = fy
    return rep_force

def getLaserInfo(rho, angle_increment, angle_min, index):
    global x_n, y_n, x_goal, y_goal
    theta = index*angle_increment + angle_min + pi
    x = rho*np.cos(theta) + x_n
    y = rho*np.sin(theta) + y_n
    d_followed = calcDistance(x,y,x_goal,y_goal)
    return (rho, theta, x, y, d_followed)

def callback_laser(data):
    global v_fw, w_z, x_goal, y_goal, x_n, y_n, L, inflation, state, max_range
    laserData = []
    discontinuities = []
    angle_increment = np.around(data.angle_increment,4)
    angle_min = data.angle_min
    max_range = data.range_max
    
    #if isGoalFree(data):
    	#state = 'MOVE_TO_GOAL'
    	#heuristic(laserData, True)
    	
    #print('Free', isGoalFree(data))
    for index in range(len(data.ranges)-1):
        rho = data.ranges[index]
        theta = index*angle_increment + angle_min
        laserInfo = getLaserInfo(data.ranges[index+1], data.angle_increment, data.angle_min, index+1)
        laserData.append(getLaserInfo(data.ranges[index], data.angle_increment, data.angle_min, index))
        if(data.ranges[index] != max_range):
            if(data.ranges[index+1] < data.ranges[index]):
                discontinuities.append(getLaserInfo(data.ranges[index+1], data.angle_increment, data.angle_min, index+1))
            elif(data.ranges[index-1] < data.ranges[index]):
                discontinuities.append(getLaserInfo(data.ranges[index-1], data.angle_increment, data.angle_min, index-1))
    if isGoalFree(data):
    	state = 'MOVE_TO_GOAL'
    	heuristic(laserData, True)
    	print('MOVE_TO_GOAL')	
    elif(len(discontinuities) > 0):
        print('MOVE_TO_Oi')
        state = 'MOVE_TO_O'
        heuristic(discontinuities, False)
        print (discontinuities)
    #else:
        #state = 'MOVE_TO_GOAL'
        #heuristic(laserData, True)
    #print(v_fw, w_z)

def heuristic(laserData, moveToGoal):
    global Kp, Usat, v_fw, w_z, x_goal, y_goal, goToGoal, min_heuristic
    if(not moveToGoal):
        #print('moveToO')
        minDist = laserData[0][4]
        minIdx = 0
        for i in range(len(laserData)):
            dist = laserData[i][0] + laserData[i][4]
            if(laserData[i][2] < x_goal + 0.2 and laserData[i][2] > x_goal - 0.2 and laserData[i][3] < y_goal + 0.2 and laserData[i][3] > y_goal - 0.2):
                goToGoal = True
            #print('i:', i, 'dist: ',dist)
            if(dist < minDist):
                minDist = dist
                minIdx = i
        if(minDist < min_heuristic):
            min_heuristic = minDist
        if(goToGoal):
            #[x_ref, y_ref, Vx_ref, Vy_ref] = refference_trajectory(x_goal, y_goal)
            x_ref = x_goal
            y_ref = y_goal
            #print('goToGoal')
        else:
            #print('goToLaser: ', laserData[minIdx][2], laserData[minIdx][3])
            #[x_ref, y_ref, Vx_ref, Vy_ref] = refference_trajectory(laserData[minIdx][2], laserData[minIdx][3])
            x_ref = laserData[minIdx][2]
            y_ref = laserData[minIdx][3]
        #print(x_ref, y_ref)
        attr_force = calcAttractiveForce(x_ref, y_ref)
        rep_force = calcRepulsiveForces(laserData)
        Vx_r = attr_force[0] + rep_force[0]
        Vy_r = attr_force[1] + rep_force[1]
        [v_fw, w_z] = feedback_linearization(Vx_r, Vy_r)
    else:
        attr_force = calcAttractiveForce(x_goal, y_goal)
        rep_force = calcRepulsiveForces(laserData)
        Vx_r = attr_force[0] + rep_force[0]
        Vy_r = attr_force[1] + rep_force[1]
        [v_fw, w_z] = feedback_linearization(Vx_r, Vy_r)
        print(v_fw, w_z)
    #print('Idx:',minIdx, ' laser XY: ', laserData[minIdx][2], laserData[minIdx][3])
    



def refference_trajectory(x_goal, y_goal):
    x_ref = x_goal
    y_ref = y_goal
    Vx_ref = 0
    Vy_ref = 0
    return (x_ref, y_ref, Vx_ref, Vy_ref)

# Rotina para a geracao da entrada de controle
def trajectory_controller(x_ref, y_ref, Vx_ref, Vy_ref, Kp, Usat):
    global x_n, y_n, theta_n
    Ux = Vx_ref + Kp * (x_ref - x_n)
    Uy = Vy_ref + Kp * (y_ref - y_n)
    absU = sqrt(Ux ** 2 + Uy ** 2)
    if (absU > Usat):
        Ux = Usat * Ux / absU
        Uy = Usat * Uy / absU
    return (Ux, Uy)


# Rotina feedback linearization
def feedback_linearization(Ux, Uy):
    global x_n, y_n, theta_n
    global d
    VX = cos(theta_n) * Ux + sin(theta_n) * Uy
    WZ = (-sin(theta_n) / d) * Ux + (cos(theta_n) / d) * Uy
    return (VX, WZ)

# Rotina primaria
def example():
    global x_n, y_n, theta_n, x_goal, y_goal, v_fw, w_z
    rospy.init_node("Q1_node") #inicializa o no "este no"
    pub_stage = rospy.Publisher("/cmd_vel", Twist, queue_size=1)  #declaracao do topico para comando de velocidade
    rospy.Subscriber("/base_pose_ground_truth", Odometry, callback_pose) #declaracao do topico onde sera lido o estado do robo
    rospy.Subscriber("/base_scan", LaserScan, callback_laser)

    #Define uma variavel que controlar a a frequencia de execucao deste no
    rate = rospy.Rate(freq)
    vel = Twist()
    sleep(0.2)
    
    i = 0
    a = 3 
    # O programa do no consiste no codigo dentro deste while
    while not rospy.is_shutdown(): #"Enquanto o programa nao ser assassinado"
        # Incrementa o tempo
        
        i = i + 1

        vel.linear.x = v_fw
        vel.angular.z = w_z
        pub_stage.publish(vel)

        #Espera por um tempo de forma a manter a frequencia desejada
        rate.sleep()

# Funcao inicial
if __name__ == '__main__':
    try:
        x_goal, y_goal = input('(x_goal, y_goal)').split()
        x_goal, y_goal = [float(i) for i in [x_goal, y_goal]]
        example()
    except rospy.ROSInterruptException:
        pass
