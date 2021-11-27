#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import sqrt, atan2, exp, atan, cos, sin, acos, pi, asin, atan2, floor
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from time import sleep
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs.msg import LaserScan
import tf
import sys
import numpy as np
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimg

global x_n, y_n, theta_n

class Wavefront():
    def __init__(self, map, goal, start, cell_size):
        self.map = map
        self.goal = goal
        self.start = start
        self.moveHeap = []
        self.cell_size = cell_size
        self.goal2d = self.goal_to_node2d()
        #self.map[self.goal2d[0]][self.goal2d[1]] = 2
        self.map[self.goal2d[1]][self.goal2d[0]] = 2
        #print(*self.map)
        print(self.goal2d)

    def isGoalValid(self):
        if(self.map[self.goal2d[0]][self.goal2d[1]] == 1):
            return False
        elif(self.map[self.goal2d[0]][self.goal2d[1]] == 0):
            return True

    def goal_to_node2d(self):
        #goal: x,y
        goal2d = np.array([0,0])
        goal2d[0] = self.goal[0]/self.cell_size
        goal2d[1] = -self.goal[1]/self.cell_size
        return goal2d

    def node2d_to_goal(self, cell):
        x = cell[0]*self.cell_size + cell_size/2
        y = -cell[1]*self.cell_size - cell_size/2
        return (x,y)

    def searchWavefront(self):
        self.goal2d
        nextHeap = []
        x = self.goal2d[0]
        y = self.goal2d[1]
        lastwave = 3
        
        #Movimentos NORTE SUL LESTE OESTE
        moves = [(x+1,y),(x-1,y),(x,y-1),(x,y+1)]
        for move in moves:
            if(self.map[move[1]][move[0]] == 0):
                self.map[move[1]][move[0]] = lastwave
                self.moveHeap.append(move)

        for wave in range(4, 1000):
            lastwave = lastwave+1
            while(self.moveHeap != []):
                cell = self.moveHeap.pop()
                x = cell[0]
                y = cell[1]
                moves = [(x+1,y),(x-1,y),(x,y-1),(x,y+1)]
                for move in moves:
                    if(self.map[move[1]][move[0]] != 1):
                        if(self.map[move[1]][move[0]] == 0 and self.map[y][x] == wave - 1):
                            self.map[move[1]][move[0]] = wave
                            nextHeap.append(move)
                            #print('appending to nextheap', nextHeap)
                        if(move[0] == self.start[0] and move[1] == self.start[1]):
                            return self.map, lastwave
            if not nextHeap:
                print('Alvo não encontrado após explorar o mapa')
                return self.map, 1
            self.moveHeap = nextHeap
            nextHeap = []

    def plan(self, lastwave):
        poses = []
        pos = self.start
        poses.append(pos)
        finished = False
        #self.map[self.start[0]][self.start[1]] = 'R'

        while(not finished):
            print(lastwave)
            x = pos[0]
            y = pos[1]
            moves = [(x+1,y),(x-1,y),(x,y-1),(x,y+1)]
            for i in range(len(moves)):
                move = moves[i]
                if(self.map[move[1]][move[0]] == lastwave - 1 ):
                    nextIndex = i
                elif(self.map[move[1]][move[0]] == 2):
                    finished = True
                    nextIndex = i
            lastwave = lastwave - 1
            pos = moves[nextIndex]
            poses.append(pos)
            #self.map[pos[0]][pos[1]] = 'L'
        return poses
            


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
def feedback_linearization(Ux, Uy, d):
    global x_n, y_n, theta_n
    VX = cos(theta_n) * Ux + sin(theta_n) * Uy
    WZ = (-sin(theta_n) / d) * Ux + (cos(theta_n) / d) * Uy
    return (VX, WZ)

def readImage(cell_size):
    fig = plt.figure(figsize=(8,8), dpi=100)
    #ax = fig.add_subplot(111, aspect='equal')

    # Invertendo os valores para visualização (Branco - 0, Preto - 1)
    img = 1 - mpimg.imread('../worlds/map_1.png')
    #img = 1 - mpimg.imread('img/maze.png')

    # Apenas para garantir que só teremos esses dois valores
    threshold = 0.5
    img[img > threshold] = 1
    img[img<= threshold] = 0

    #ax.imshow(img, cmap='Greys', origin='upper')
    # Dimensões do mapa informado em metros (X, Y)
    map_dims = np.array([60, 60]) # Cave 

    # Escala Pixel/Metro
    print(img.shape)
    sy, sx = img.shape[0:2] / map_dims

    # Tamanho da célula do nosso Grid (em metros)

    rows, cols = (map_dims / cell_size).astype(int)
    grid = np.zeros((rows, cols))

    # Preenchendo o Grid
    # Cada célula recebe o somatório dos valores dos Pixels
    for r in range(rows):
        for c in range(cols):
            
            xi = int(c*cell_size*sx)
            xf = int(xi + cell_size*sx)
            
            yi = int(r*cell_size*sy)
            yf = int(yi + cell_size*sy)
                        
            grid[r, c] = np.sum(img[yi:yf,xi:xf])
            
    # Binarizando as células como Ocupadas (1) ou Não-ocupadas (0)       
    grid[grid > threshold] = 1
    grid[grid<= threshold] = 0   
    #fig = plt.figure(figsize=(8,8), dpi=100)
    #ax = fig.add_subplot(111, aspect='equal')

    # Plotando Mapa e Células
    #obj = ax.imshow(img, cmap='Greys', extent=(0, map_dims[1], 0, map_dims[0]), origin='upper')
    #obj = ax.imshow(grid, cmap='Reds', extent=(0, map_dims[1], 0, map_dims[0]), alpha=.6)

    # Plotando as linhas do grid para facilitar a visualização
    #ax.grid(which='major', axis='both', linestyle='-', color='r', linewidth=1)
    #ax.set_xticks(np.arange(0, map_dims[1]+1, cell_size))
    #ax.set_yticks(np.arange(0, map_dims[0]+1, cell_size))
    return grid

def calcDistance(x_n, y_n, x_d, y_d):
    return sqrt(((x_d - x_n)**2 + (y_d - y_n)**2))


def control(poses):
    #Tempo de simulacao no stage
    global x_n, y_n
    freq = 100
    Usat = 5
    d = 0.8
    Kp = 1

    #Define uma variavel que controlar a a frequencia de execucao deste no
    rate = rospy.Rate(freq)
    vel = Twist()
    sleep(0.2)

    # O programa do no consiste no codigo dentro deste while
    for pose in poses:
        print(pose)
        x_goal = pose[0]
        y_goal = pose[1]
        # Incrementa o tempo
        dist = calcDistance(x_n,y_n,x_goal,y_goal)
        while(dist > 0.5):
            [x_ref, y_ref, Vx_ref, Vy_ref] = refference_trajectory(x_goal, y_goal)

            [Ux, Uy] = trajectory_controller(x_ref, y_ref, Vx_ref, Vy_ref, Kp, Usat)

            [V_forward, w_z] = feedback_linearization(Ux, Uy, d)

            vel.linear.x = V_forward
            vel.angular.z = w_z
            pub_stage.publish(vel)
            dist = calcDistance(x_n, y_n, x_goal, y_goal)

            #Espera por um tempo de forma a manter a frequencia desejada
            rate.sleep()

# Funcao inicial
if __name__ == '__main__':
    try:
        rospy.init_node("Q4_node") #inicializa o no "este no"
        pub_stage = rospy.Publisher("/cmd_vel", Twist, queue_size=1)  #declaracao do topico para comando de velocidade
        rospy.Subscriber("/base_pose_ground_truth", Odometry, callback_pose) #declaracao do topico onde sera lido o estado do robo
        
        cell_size = 2
        x_goal, y_goal = input('(x_goal, y_goal)').split()
        x_goal, y_goal = [float(i) for i in [x_goal, y_goal]]
        grid = readImage(cell_size)
        start_x = floor(x_n/cell_size)
        start_y = floor(-y_n/cell_size)
        print('pose: ', start_x, start_y)

        wavefront = Wavefront(grid, np.array([x_goal, y_goal]), np.array([start_x,start_y]), cell_size)
        if(not wavefront.isGoalValid()):
            print('Posicao de alvo invalida')
            exit()

        grid, wave = wavefront.searchWavefront()
        #print(grid)
        for line in grid:
            string = ' '.join(str(int(e)) for e in line)
            print('\n')
            sys.stdout.write(string)
        plan = wavefront.plan(wave)
        plan.pop()
        plan.pop()
        planConverted = []
        for cell in plan:
            pose = wavefront.node2d_to_goal(cell)
            planConverted.append(pose)
            #grid[cell[0]][cell[0]] = 'x'
        planConverted.pop()
        planConverted.append((x_goal,y_goal))
        print(plan)
        control(planConverted)
    except rospy.ROSInterruptException:
        pass
