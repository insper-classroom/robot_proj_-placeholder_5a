#! /usr/bin/env python3
# -*- coding:utf-8 -*-
from __future__ import print_function, division


### PERSEGUE O CENTRO DAS FAIXAS AMARELAS
# COM ODOMETRIA E ARUCO
# Sem usar placas (ids) pra saber o movimento que deve fazer
# Vai à pista direita sem saída, volta e faz um loop completo. Mas não anda pela pista esquerda sem saída

import tf
import cv2
import math
import rospy
import numpy as np
import cv2.aruco as aruco

from geometry_msgs.msg import Twist, Vector3, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CompressedImage
from sensor_msgs.msg import LaserScan 

from cv_bridge import CvBridge, CvBridgeError # Bridgge converte imagem de formato ros pra cv2

# import visao_module



bridge = CvBridge() 


#####################################################
# Variáveis
obj = ('blue', 11, 'dog')

# atraso = 1.5E9 # 1 segundo e meio. Em nanossegundos
DELTA = 25 # faixa aceitavel para o robo avancar

#####################################################
# FUNÇÕES Básicas

def center_of_mass(mask):
    """ Retorna uma tupla (cx, cy) que desenha o centro do contorno"""
    M = cv2.moments(mask)
    # Usando a expressão do centróide definida em: https://en.wikipedia.org/wiki/Image_moment
    if M["m00"] == 0:
        M["m00"] = 1
    cX = int(M["m10"] / M["m00"])
    cY = int(M["m01"] / M["m00"])
    return [int(cX), int(cY)]

def cross(img_rgb, point, color, width,length):
    cv2.line(img_rgb, (int( point[0] - length/2 ), point[1] ),  (int( point[0] + length/2 ), point[1]), color ,width, length)
    cv2.line(img_rgb, (point[0], int(point[1] - length/2) ), (point[0], int( point[1] + length/2 ) ),color ,width, length) 


# FUNÇÕES Maiores
# Retorna: ponto médio do maior contorno, centro do frame, area do maior contorno
def identifica_cor0(frame):
    # No OpenCV, o canal H vai de 0 até 179, logo cores similares ao  vermelho puro (H=0) estão entre H=-8 e H=8. 
    # Precisamos dividir o inRange em duas partes para fazer a detecção do vermelho:
    frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    cor_menor, cor_maior = np.array([26, 250, 250]), np.array([32, 255, 255])
    segmentado_cor = cv2.inRange(frame_hsv, cor_menor, cor_maior)

    # A operação MORPH_CLOSE fecha todos os buracos na máscara menores  que um quadrado 7x7. 
    segmentado_cor = cv2.morphologyEx(segmentado_cor,cv2.MORPH_CLOSE,np.ones((7, 7)))

    media = center_of_mass(segmentado_cor)
    cross(frame, media, [255,0,0], 1, 17)

    # Desenhamos os contornos encontrados no frame
    contornos, arvore = cv2.findContours(segmentado_cor.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) 
    cv2.drawContours(frame, contornos, -1, [0, 0, 255], 5)
    
    # Exibe a imagem do contorno segmentado
    cv2.waitKey(1)

    return media


#####################################################
# CLASSES

class Robo:
    # Atributos:
    objetivo, estado, sucesso = (), 0, 0
    scan, frame = [], []
    centro, ponto_alvo, maior_area =  (), (), 0
    posicao, id = (0,0), -1 # posição
    velocidade = Twist(Vector3(0,0,0), Vector3(0,0,0))
    bifY, bifT = (),() # Ponto Bifurcação Y (ID=100), Ponto Bifurcação Y (ID=50)
    # Publishers
    pub_velocidade = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)
    
    # Função executada ao criar objeto
    def __init__(self, objetivo):
        # Armazena o centro do frame
        self.centro = (320,240)
        self.objetivo = objetivo
        # Declara os Subscribers e seus respectivos callback
        sub_scan = rospy.Subscriber("/scan", LaserScan, self.callback_scan) 
        sub_camera = rospy.Subscriber("camera/image/compressed", CompressedImage, self.callback_camera, queue_size=4, buff_size = 2**24)
        odom_sub = rospy.Subscriber('/odom', Odometry, self.callback_odometria)
        # Gira 180 graus
        # vel = Twist(Vector3(0,0,0), Vector3(0,0,math.pi/5))    
        # for i in range(5):
        #     self.pub_velocidade.publish(vel)
        #     rospy.sleep(1)

    # Callback ao receber scan
    def callback_scan(self, dado):
        ranges = np.array(dado.ranges).round(decimals=2)
        self.scan = ranges
    
    # Callback ao receber imagem da câmera
    def callback_camera(self, imagem):
        self.frame = imagem
        cv_image = imagem

        try:
            cv_image = bridge.compressed_imgmsg_to_cv2(imagem, "bgr8")

            # SALVA ID mais proximo
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            aruco_dict  = aruco.getPredefinedDictionary(aruco.DICT_6X6_250) ######################################
            corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict) #, parameters=parameters)
            # Desenha os IDS na imagem
            aruco.drawDetectedMarkers(cv_image, corners, ids)          


            if ids is not None:
                self.id = ids[0][0]


            media =  identifica_cor0(cv_image)

            img = np.array(cv_image)

            # Armazena o frame no objeto e mostra o frame recebido
            self.frame = img
            self.ponto_alvo = media
            cv2.imshow("Camera", img)

        except CvBridgeError as e:
            print('ex', e)
        
    # Callback da odometria
    def callback_odometria(self, data):
        x = data.pose.pose.position.x
        y = data.pose.pose.position.y
        self.posicao = (x,y)

    # Função para alterar a velocidade vel_lin e vel_ang são listas de 3 elementos
    def veloc(self, vel_lin, vel_ang):
        linX, linY, linZ = vel_lin
        angX, angY, angZ = vel_ang
        self.velocidade = Twist(Vector3(linX, linY, linZ), Vector3(angX, angY, angZ))

    # Função para perseguir o ponto armazenado
    def perseguir_ponto(self):
        if len(self.ponto_alvo) != 0 and len(self.centro) != 0:
            # print('perseguindo o ponto', self.ponto_alvo)

            # Se a camera tiver praticamente alinhada com o objeto alvo, então o robô anda pra frente
            if abs(self.ponto_alvo[0] - self.centro[0]) < DELTA:
                self.veloc([0.1,0,0],[0,0,0])	# Anda o robô pra frente
            
            # Se a cruz da mira não tiver alinhada com o alvo, gira o robô pra alinhar com ela
            else:
                if (self.ponto_alvo[0] > self.centro[0]):
                    self.veloc([0,0,0.1],[0,0,-0.1]) # girar para esquerda do robô (sentido anti-horário)
                if (self.ponto_alvo[0] < self.centro[0]):
                    self.veloc([0,0,0],[0,0,0.1]) # girar para direita do robô (sentido horário)

    # Função executada ao final de cada iteração do while
    def loop(self):
        # Se já tiver recebido os dados do LIDAR
        if len(self.scan) != 0:

            # Distância até o objeto à frente (com 10 graus pra cada lado)
            sc = list(self.scan)
            distancia = np.min(sc[:10]+sc[-10:])

            print('')
            print('Posição Atual:', self.posicao)
            print('Scan de proximidade: ', distancia, 'm')
            print('Último ID dectado:', self.id)
            
            # Se o robô não tiver perto demais de algo na frente dele
            if self.scan[0] > 0.2:
                # Se o objeto tiver armazenado um ponto_alvo
                self.perseguir_ponto()
            else:
                self.veloc([0,0,0],[0,0,0])

            # Se chegar perto da placa bifY
            if self.id == 100 and self.scan[0] < 1.2:
                print('Indo pro caminho da esquerda...')
                # Armazena a posição atual como
                self.bifY = self.posicao
                
                # | Vou ter que fazer ele andar pra frente enquanto faz a curva pra já começar a medir a distancia 
                # Gira 30 graus
                vel = Twist(Vector3(0.1,0,0), Vector3(0,0,math.pi/18))    
                for i in range(6):
                    self.pub_velocidade.publish(vel)
                    rospy.sleep(1)

            # Se tiver a menos de 0.5m da placa da bifT (ID = 200)
            if self.id == 200 and self.scan[0] < 1.1:
                # print('Ponto BifT...')
                # Armazena a posição atual como
                self.bifT= self.posicao
                
                # | Vou ter que fazer ele andar pra frente enquanto faz a curva pra já começar a medir a distancia 
                # Gira 60 graus
                vel = Twist(Vector3(0.1,0,0), Vector3(0,0,math.pi/18))    
                for i in range(6):
                    self.pub_velocidade.publish(vel)
                    rospy.sleep(1)

            # Se tiver num beco sem saída gira 180
            if (self.id == 50 or self.id == 150) and self.scan[0] < 1:
                # Gira 180 graus
                vel = Twist(Vector3(0,0,0), Vector3(0,0,math.pi/5))    
                for i in range(5):
                    self.pub_velocidade.publish(vel)
                    rospy.sleep(1)

                # Se tiver ido pra pista sem saída da esquerda sem usar a odometria, então apaga bifY armazenado
                if self.id == 50: 
                    self.bifY = ()



        # Se tiver gravado um bifY
        if len(self.bifY) != 0:
            # print('Posição BifY:', self.bifY)
            y_bif,x_bif = self.bifY
            y,x = self.posicao
            dist = math.sqrt((x-x_bif)**2+(y-y_bif)**2)
            print('Distancia até bifY:', dist)
            
            # Se tiver muito próximo desse ponto 
            if dist < 0.4:
                print('Indo pro caminho da direita...')

                # Gira 90 graus
                vel = Twist(Vector3(0,0,0), Vector3(0,0,math.pi/18))    
                for i in range(9):
                    self.pub_velocidade.publish(vel)
                    rospy.sleep(1)
                
                # Apaga o valor do ponto salvo
                self.bifY = ()

        # Se tiver gravado bifT
        if len(self.bifT) != 0:
            # print('Posição BifT:', self.bifT)
            y_bif,x_bif = self.bifT
            y,x = self.posicao
            dist = math.sqrt((x-x_bif)**2+(y-y_bif)**2)
            print('Distancia até bifT:', dist)
            
            # Se tiver muito próximo desse ponto 
            if dist < 0.25:
                print('Saindo do círculo...')

                # Gira 90 graus
                vel = Twist(Vector3(0,0,0), Vector3(0,0,math.pi/18))    
                for i in range(9):
                    self.pub_velocidade.publish(vel)
                    rospy.sleep(1)
                
                # Apaga o valor do ponto salvo
                self.bifT = ()

        # Publicar velocidade do objeto no robô
        self.pub_velocidade.publish(self.velocidade)
        rospy.sleep(0.1)
        

#################################################################################
# CÓDIGOS

if __name__=="__main__":
    rospy.init_node("Projeto") #  Nome do script
    robo = Robo(obj) # Cria novo objeto Robo e fornece o objetivo pra ele
 
    try:
        while not rospy.is_shutdown():
            robo.loop() # Loop do robo 

    except rospy.ROSInterruptException:
        print("Ocorreu uma exceção com o rospy")