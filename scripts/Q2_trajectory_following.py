#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import sqrt, atan2, exp, atan, cos, sin, acos, pi, asin, atan2
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from time import sleep
from visualization_msgs.msg import Marker, MarkerArray
import tf
import sys

# Frequencia de simulacao no stage
global freq
freq = 100  # Hz

# Velocidade de saturacao
global Usat
Usat = 5

# Estados do robo
global x_n, y_n, theta_n
x_n = 0.0  # posicao x atual do robo
y_n = 0.0  # posicao y atual do robo
theta_n = 0.0  # orientacao atual do robo

# Estados do robo
# global x_goal, y_goal
# x_goal = 0
# y_goal = 0

# Relativo ao feedback linearization
global d
d = 0.80

# Relativo ao controlador (feedforward + ganho proporcional)
global Kp
Kp = 10

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

# Rotina para a geracao da trajetoria de referencia
# def refference_trajectory(time):
##MUDAR PARA CIRCULO
    # global x_goal, y_goal
    # x_ref = x_goal
    # y_ref = y_goal
    # Vx_ref = 0
    # Vy_ref = 0
    # return (x_ref, y_ref, Vx_ref, Vy_ref)

# Rotina para a geracao da entrada de controle

def trajectory_controller(x_ref, y_ref, Vx_ref, Vy_ref):
    global x_n, y_n, theta_n
    global Kp
    global Usat
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
    global freq
    global x_n, y_n, theta_n
    global pub_rviz_ref, pub_rviz_pose
    
    rospy.init_node("Q2_node") #inicializa o no "este no"
    pub_stage = rospy.Publisher("/cmd_vel", Twist, queue_size=1)  #declaracao do topico para comando de velocidade
    rospy.Subscriber("/base_pose_ground_truth", Odometry, callback_pose) #declaracao do topico onde sera lido o estado do robo
    
    #Inicializa os nos para enviar os marcadores para o rviz
    pub_rviz_ref = rospy.Publisher("/visualization_marker_ref", Marker, queue_size=1) #rviz marcador de velocidade de referencia
    pub_rviz_pose = rospy.Publisher("/visualization_marker_pose", Marker, queue_size=1) #rviz marcador de velocidade do robo
    
    #Define uma variavel que controlar a a frequencia de execucao deste no
    rate = rospy.Rate(freq)
    vel = Twist()
    sleep(0.2)

    #a = input('Digite os semieixos (a)')
    #a = float(a)
    #a, b, r = input('(a, b, r)').split()
    #a, b, r = [float(i) for i in [a, b, r]]
    
    i = 0
    a = 3 
    # O programa do no consiste no codigo dentro deste while
    while not rospy.is_shutdown(): #"Enquanto o programa nao ser assassinado"
        # Incrementa o tempo
        
        i = i + 1
        t = i / float(freq)

        # Obtem a trajetoria de referencia
        x_ref = a * sin(t)
        y_ref = a * sin(t)*cos(t)
        #print(x_ref,y_ref)
        
        Vx_ref = a * cos(t)
        Vy_ref = a * cos(2*t)
        print(Vx_ref,Vy_ref)
        
        # circulo
        #x_ref = a + r*cos(t)
        #y_ref = b + r*sin(t)
        
        
        #circulo
        #Vx_ref = - r * sin(t)
        #Vy_ref =   r * cos(t)
        
        # Aplica o controlador
        # Vx_ref = Kp * (x_ref - x_n) - a * sin(cx*t) * cx
        # Vy_ref = Kp * (y_ref - y_n) + b * cos(cy*t) * cy

        # Aplica o feedback linearization
        # vel.linear.x = cos(theta_n) * Vx_ref + sin(theta_n) * Vy_ref
        # vel.angular.z = -(sin(theta_n) * Vx_ref) / d + (cos(theta_n) * Vy_ref) / d

        #Aplica o controlador
        [Ux, Uy] = trajectory_controller(x_ref, y_ref, Vx_ref, Vy_ref)

        # Aplica o feedback linearization
        [V_forward, w_z] = feedback_linearization(Ux, Uy)
        #if (V_forward < 0):
            #V_forward = V_forward*(-1)
	
        # Publica as velocidades
        vel.linear.x = V_forward
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
