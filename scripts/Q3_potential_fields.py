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
global Ka, Kr
Ka = 1
Kr = 1000.0

global v_fw, w_z
v_fw = 0.0
w_z = 0.0

global min_d
min_d = 14.0

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

def calcAttractiveForce():
    global x_goal, y_goal, Ka
    dx = x_goal - x_n
    dy = y_goal - y_n
    rho = np.array([dx,dy])
    norm_rho = np.sqrt(dx**2 + dy**2)
    if(norm_rho <= 0.5):
        attr_force = Ka*rho
    else:
        attr_force = Ka*rho/norm_rho
    return attr_force

def calcRepulsiveForces(laserData):
    global Kr, min_d
    #print(laserData)
    rep_force = np.array([0.0,0.0])
    for dist, theta, x, y in laserData:
        if dist <= min_d:
            fx = Kr*(2*x*(min_d - dist)/(min_d*dist**4))
            fy = Kr*(2*y*(min_d - dist)/(min_d*dist**4))
            #fx = Kr*(1/(range**2))
            #fy = Kr*(1/(range**2))
            print("fx",fx, "fy ",fy)
            #rep_force[0] += 
            #rep_force[1] += rep_force[1]+fy
            rep_force[0] = fx
            rep_force[1] = fy
    return rep_force

def callback_laser(data):
    global v_fw, w_z
    laserData = []
    angle_increment = np.around(data.angle_increment,4)
    for index in range(len(data.ranges)):
        rho = data.ranges[index]
        theta = index*angle_increment
        x = rho*np.cos(theta)
        y = rho*np.sin(theta)
        laserData.append((rho, theta, x, y))
    attr = calcAttractiveForce()
    repulsive = calcRepulsiveForces(laserData)
    resulting_force = attr + repulsive
    print('repulsive', repulsive, 'attractive', attr)
    v_fw, w_z = feedback_linearization(resulting_force[0], resulting_force[1])
    #print(v_fw, w_z)
    

# Rotina feedback linearization
def feedback_linearization(Ux, Uy):
    global x_n, y_n, theta_n
    global d
    VX = cos(theta_n) * Ux + sin(theta_n) * Uy
    WZ = (-sin(theta_n) / d) * Ux + (cos(theta_n) / d) * Uy
    return (VX, WZ)

# Rotina primaria
def example():
    global x_n, y_n, theta_n
    global pub_rviz_ref, pub_rviz_pose
    global x_goal, y_goal
    x_goal, y_goal = input('(x_goal, y_goal)').split()
    x_goal, y_goal = [float(i) for i in [x_goal, y_goal]]
    
    rospy.init_node("Q3_node") #inicializa o no "este no"
    pub_stage = rospy.Publisher("/cmd_vel", Twist, queue_size=1)  #declaracao do topico para comando de velocidade
    rospy.Subscriber("/base_pose_ground_truth", Odometry, callback_pose) #declaracao do topico onde sera lido o estado do robo
    rospy.Subscriber("/base_scan", LaserScan, callback_laser)
    
    #Inicializa os nos para enviar os marcadores para o rviz
    pub_rviz_ref = rospy.Publisher("/visualization_marker_ref", Marker, queue_size=1) #rviz marcador de velocidade de referencia
    pub_rviz_pose = rospy.Publisher("/visualization_marker_pose", Marker, queue_size=1) #rviz marcador de velocidade do robo
    
    #Define uma variavel que controlar a a frequencia de execucao deste no
    rate = rospy.Rate(freq)
    vel = Twist()
    sleep(0.2)

    #a = input('Digite os semieixos (a)')
    #a = float(a)
    
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
        example()
    except rospy.ROSInterruptException:
        pass
