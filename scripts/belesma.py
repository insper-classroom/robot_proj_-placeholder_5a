#! /usr/bin/env python3
# -*- coding:utf-8 -*-
from __future__ import print_function, division


### PERSEGUE O CENTRO DAS FAIXAS AMARELAS
# COM ODOMETRIA E ARUCO
# Sem usar placas (ids) pra saber o movimento que deve fazer
# Vai à pista direita sem saída, volta e faz um loop completo. Mas não anda pela pista esquerda sem saída

# import tf
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
# Para fazer o robô não atacar creeper basta colocar obj = ()
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

#####################################################
# CLASSES

class Robo:
    # Atributos:
    # objetivo = ()
    objetivo = ('blue',11,'dog')
    sucesso = 0
    scan, frame = [], []
    centro = (320,240)
    posicao, id = (0,0), -1 # posição
    velocidade = Twist(Vector3(0,0,0), Vector3(0,0,0))
    bifY, bifT = (),() # Ponto Bifurcação Y (ID=100), Ponto Bifurcação T (ID=200)
    ponto_alvo, ponto_pista, ponto_creeper = (), (), ()

    # Publishers
    pub_velocidade = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)
    
    # Função executada ao criar objeto
    def __init__(self):
        # Declara os Subscribers e seus respectivos callback
        sub_scan = rospy.Subscriber("/scan", LaserScan, self.callback_scan) 
        sub_camera = rospy.Subscriber("camera/image/compressed", CompressedImage, self.callback_camera, queue_size=4, buff_size = 2**24)
        odom_sub = rospy.Subscriber('/odom', Odometry, self.callback_odometria)

        # Girar 180
        # vel = Twist(Vector3(0,0,0), Vector3(0,0,math.pi/6))    
        # for i in range(6):
        #     self.pub_velocidade.publish(vel)
        #     rospy.sleep(1)

    # Callback ao receber scan
    def callback_scan(self, dado):
        ranges = np.array(dado.ranges).round(decimals=2)
        self.scan = list(ranges)

    # Callback da odometria
    def callback_odometria(self, data):
        x = data.pose.pose.position.x
        y = data.pose.pose.position.y
        self.posicao = (x,y)

    # Callback ao receber imagem da câmera
    def callback_camera(self, imagem):
        self.frame = imagem
        cv_image = imagem

        try:
            cv_image = bridge.compressed_imgmsg_to_cv2(imagem, "bgr8")

            # Armazena o ID mais proximo no objeto
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            aruco_dict  = aruco.getPredefinedDictionary(aruco.DICT_6X6_250) 
            corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict) 
            aruco.drawDetectedMarkers(cv_image, corners, ids) # Desenha os IDS na imagem
            if ids is not None:
                self.id = ids[0][0]

            self.frame = cv_image

            # Grava no objeto o ponto alvo da pista se ver ela
            self.encontrar_pista()


            # Se tiver objetivo que faz isso
            if len(self.objetivo) != 0:
                # Grava no objeto o ponto alvo do creeper se ver ele
                self.encontrar_creeper()

            img = np.array(cv_image)

            cv2.imshow("Camera", img)
            cv2.waitKey(1)

        except CvBridgeError as e:
            print('ex', e)
    

    # Se ver pista amarela, grava o ponto médio dela como alvo no objeto. Se não encontrar, retorna ()
    def encontrar_pista(self):
        frame_hsv = cv2.cvtColor(self.frame, cv2.COLOR_BGR2HSV) # No OpenCV, o canal H vai de 0 até 179,

        # Cor da faixa amarela da pista
        cor_inicial, cor_final = np.array([26, 250, 250]), np.array([32, 255, 255])
        segmentado_cor = cv2.inRange(frame_hsv, cor_inicial, cor_final)

        # A operação MORPH_CLOSE fecha todos os buracos na máscara menores  que um quadrado 7x7. 
        segmentado_cor = cv2.morphologyEx(segmentado_cor,cv2.MORPH_CLOSE,np.ones((7, 7)))

        media = center_of_mass(segmentado_cor)
        cross(self.frame, media, [255,0,0], 1, 17)

        # Desenhamos os contornos encontrados no frame
        contornos, arvore = cv2.findContours(segmentado_cor.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) 
        cv2.drawContours(self.frame, contornos, -1, [0, 0, 255], 5)
        
        # Grava o ponto que vai ser perseguido 
        self.ponto_pista = media

    # Retorna 0 se não tiver creeper na tela ou ele tiver muito longe. Se não, retorna 1 e armazena o CM do creeper como alvo
    # Se ver um creeper (com área o suficente pra parecer perto) grava o ponto médio dele como alvo
    def encontrar_creeper(self):
        # No OpenCV, o canal H vai de 0 até 179, logo cores similares ao vermelho puro (H=0) estão entre H=-8 e H=8. 
        frame_hsv = cv2.cvtColor(self.frame, cv2.COLOR_BGR2HSV)
 
        if self.objetivo[0] == 'blue':
            cor_inicial, cor_final = np.array([90, 140, 250]), np.array([100, 180, 255]) # Cor creeper azul
        
        segmentado_cor = cv2.inRange(frame_hsv, cor_inicial, cor_final)

        # Encontramos os contornos na máscara e selecionamos o de maior área
        contornos, arvore = cv2.findContours(segmentado_cor.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) 

        # Encontramos qual o contorno de maior área e sua área
        maior_contorno, maior_contorno_area = None, 0
        for cnt in contornos:
            area = cv2.contourArea(cnt)
            if area > maior_contorno_area:
                maior_contorno = cnt
                maior_contorno_area = area

        # Encontramos o centro do contorno fazendo a média de todos seus pontos.
        if not maior_contorno is None :
            maior_contorno = np.reshape(maior_contorno, (maior_contorno.shape[0], 2))
            media = maior_contorno.mean(axis=0)
            media = media.astype(np.int32)

        # print(maior_contorno_area)
        # Se o creeper tiver mais do que a área minima, então atacar o creeper
        if maior_contorno_area > 20:
            self.ponto_creeper = media
        else:
            self.ponto_creeper = ()


    # Função para alterar a velocidade vel_lin e vel_ang são listas de 3 elementos
    def veloc(self, vel_lin, vel_ang):
        linX, linY, linZ = vel_lin
        angX, angY, angZ = vel_ang
        self.velocidade = Twist(Vector3(linX, linY, linZ), Vector3(angX, angY, angZ))

    # Função para perseguir o ponto armazenado
    def perseguir_ponto(self):
        # print(abs(self.ponto_alvo[0]-self.centro[0]))
        self.veloc([0,0,0],[0,0,0])

        # Distância mínima até o objeto à frente (com 10 graus pra cada lado)
        distancia = np.min(self.scan[:10]+self.scan[-10:])
        

        if len(self.ponto_alvo) != 0:
            # Usado pra determinar se a camera do robo está ou não alinhada com o alvo
            dif = abs(self.ponto_alvo[0] - self.centro[0]) 
            
            # Se a camera tiver alinhada com o alvo e não estiver muito proximo
            if (dif < DELTA) and (distancia > 0.2): # Impede que o robô bata em algo
                self.veloc([0.2,0,0],[0,0,0])	
            
            # Se a cruz da mira não tiver alinhada com o alvo, gira o robô pra alinhar com ela
            else:
                if (self.ponto_alvo[0] >= self.centro[0]):
                    self.veloc([0.2,0,0],[0,0,-0.15]) # girar para esquerda do robô (sentido anti-horário)
                if (self.ponto_alvo[0] < self.centro[0]):
                    self.veloc([0.2,0,0],[0,0,0.15]) # girar para direita do robô (sentido horário)

        # Publicar (Executar) velocidade armazenada no no robô 
        self.pub_velocidade.publish(self.velocidade)
        rospy.sleep(0.1)
            

    # Faz o robô permanecer seguindo a faixa amarela
    def seguir_pista(self):

            # Distância mínima até o objeto à frente (com 10 graus pra cada lado)
            distancia = np.min(self.scan[:10]+self.scan[-10:])

           # Se chegar perto da placa bifY
            if self.id == 100 and distancia < 1.2:
                print('Indo pro caminho da esquerda...')
                # Armazena a posição atual como
                self.bifY = self.posicao
                
                # | Vou ter que fazer ele andar pra frente enquanto faz a curva pra já começar a medir a distancia 
                # Gira 30 graus
                vel = Twist(Vector3(0.1,0,0), Vector3(0,0,math.pi/18))    
                for i in range(6):
                    self.pub_velocidade.publish(vel)
                    rospy.sleep(1)

            # Se tiver perto da placa da bifT (ID = 200)
            if self.id == 200 and distancia < 1.1:
                print('Placa: Bifurcação T')
                # Armazena a posição atual
                self.bifT= self.posicao
                
                # | Vou ter que fazer ele andar pra frente enquanto faz a curva pra já começar a medir a distancia 
                # Gira 90 graus
                vel = Twist(Vector3(0.2,0,0), Vector3(0,0,math.pi/8))    
                for i in range(4):
                    self.pub_velocidade.publish(vel)
                    rospy.sleep(1)

            # Se tiver num beco sem saída gira 180
            if (self.id == 50 or self.id == 150) and distancia < 1:
                print('Placa: Rua sem saída')
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
                print('Placa: Bifurcação Y')
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
                if dist < 0.60:
                    print('Saindo do balão...')

                    # Gira 90 graus
                    vel = Twist(Vector3(0.15,0,0), Vector3(0,0,math.pi/8))    
                    for i in range(4):
                        self.pub_velocidade.publish(vel)
                        rospy.sleep(1)
                    
                    # Apaga o valor do ponto salvo
                    self.bifT = ()

 
    # Função executada ao final de cada iteração do while
    def loop(self):

        if len(self.scan) != 0:
            # Distância mínima até o objeto à frente (com 10 graus pra cada lado)
            distancia = np.min(self.scan[:10]+self.scan[-10:])
            
            print('')
            print('Posição Atual:', self.posicao)
            print('Scan de proximidade: ', distancia)
            print('Último ID dectado:', self.id)

            # Persegue creeper
            if len(self.ponto_creeper)!= 0 and self.sucesso == 0:
                self.ponto_alvo = self.ponto_creeper

                # Se tiver muito proximo do creeper diz que cumpriu a missão
                if distancia < 0.3:
                    self.sucesso = 1
                    self.ponto_alvo = self.ponto_pista

                    # Girar 180
                    vel = Twist(Vector3(0,0,0), Vector3(0,0,math.pi/6))    
                    for i in range(6):
                        self.pub_velocidade.publish(vel)
                        rospy.sleep(1)

            # Persegue faixa amarela
            elif len(self.ponto_pista)!= 0:
                self.ponto_alvo = self.ponto_pista
                self.seguir_pista()
        
            self.perseguir_ponto()
 

#################################################################################
# CÓDIGOS

if __name__=="__main__":
    rospy.init_node("Projeto") #  Nome do script
    robo = Robo() # Cria novo objeto Robo
 
    try:
        while not rospy.is_shutdown():
            robo.loop() # Loop do robo 

    except rospy.ROSInterruptException:
        print("Ocorreu uma exceção com o rospy")