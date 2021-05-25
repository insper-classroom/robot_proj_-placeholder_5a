#! /usr/bin/env python3
# -*- coding:utf-8 -*-

import cv2
import math
import rospy
import numpy as np
import cv2.aruco as aruco

from std_msgs.msg import Float64
from geometry_msgs.msg import Twist, Vector3, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CompressedImage
from sensor_msgs.msg import LaserScan 
from tf import transformations
from cv_bridge import CvBridge, CvBridgeError # Bridgge converte imagem de formato ros pra cv2

bridge = CvBridge() 


#####################################################
# Variáveis
# Para fazer o robô não atacar creeper basta colocar obj = ()
obj = ('red', 11, 'dog')
# obj = ()

# Área mínima que da cor segmentada do creeper para perseguir ele
area_min_creeper = 700

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

#####################################################
# CLASSES

class Robo:
    # Atributos:
    objetivo = obj
    sucesso = 0
    scan, frame_modificado, frame_original = [], [], []
    centro, id, distancia = (320,240), -1, -1
    placas_obedecidas, debug = [], []
   
    # Função executada ao criar objeto
    def __init__(self):
        # Declara os Subscribers e seus respectivos callbacks
        sub_scan = rospy.Subscriber("/scan", LaserScan, self.callback_scan) 
        sub_camera = rospy.Subscriber("camera/image/compressed", CompressedImage, self.callback_camera, queue_size=4, buff_size = 2**24)
        odom_sub = rospy.Subscriber('/odom', Odometry, self.callback_odometria)

        # Declara os Publishers
        self.pub_velocidade = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)
        self.pub_ombro = rospy.Publisher("/joint1_position_controller/command", Float64, queue_size=1)
        self.pub_garra = rospy.Publisher("/joint2_position_controller/command", Float64, queue_size=1)

        # Velocidade do robô
        self.velocidade = Twist(Vector3(0,0,0), Vector3(0,0,0))

        # Posição usando odometria em relação à posição inicial do robô. 
        self.posicao, self.bifY, self.bifT = (0,0,0),(),() 
        # Ponto correspondente à um pixel na tela
        self.ponto_alvo, self.ponto_pista, self.ponto_creeper = (), (), ()

        # Gira 180 graus
        print("Girando 180...")
        vel = Twist(Vector3(0,0,0), Vector3(0,0,math.pi/5))    
        for i in range(5):
            self.pub_velocidade.publish(vel)
            rospy.sleep(1)

    # Callback ao receber scan
    def callback_scan(self, dado):
        ranges = np.array(dado.ranges).round(decimals=2)
        self.scan = list(ranges)
         # Distância mínima até o objeto à frente (com 20 graus pra cada lado)
        self.distancia = np.min(self.scan[:20]+self.scan[-20:])

    # Callback da odometria
    def callback_odometria(self, data):
        # Angulo em relação ao sentido que o robô apontava na posição inicial
        quat = data.pose.pose.orientation
        lista = [quat.x, quat.y, quat.z, quat.w]
        ang = np.degrees(transformations.euler_from_quaternion(lista))[2]

        x = data.pose.pose.position.x
        y = data.pose.pose.position.y

        self.posicao = (x,y, ang)

    # Callback ao receber imagem da câmera
    def callback_camera(self, imagem):
        cv_image = imagem

        try:
            cv_image = bridge.compressed_imgmsg_to_cv2(imagem, "bgr8")
            self.frame_original = cv_image.copy()
            self.frame_modificado = cv_image

            # Coloca Aruco na imagem
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            aruco_dict  = aruco.getPredefinedDictionary(aruco.DICT_6X6_250) 
            corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict) 
            aruco.drawDetectedMarkers(self.frame_modificado, corners, ids) # Desenha os IDS na imagem 

            self.encontrar_pista() # Grava no objeto o ponto alvo da pista se ver ela

            # Se tiver objetivo e ainda não completou
            if len(self.objetivo) != 0 and self.sucesso==0:
                self.encontrar_creeper() # Grava no objeto o ponto alvo do creeper se ver ele
                            
            cross(self.frame_modificado, self.ponto_alvo, [255,0,0], 1, 17)
            # Exibe a imagem que tá no frame
            # cv2.imshow("Camera", np.array(self.frame_original))
            cv2.imshow("Segmentado do Alvo (pista ou creeper)", np.array(self.debug))
            cv2.imshow("Modificado", np.array(self.frame_modificado))
            cv2.waitKey(1)

        except CvBridgeError as e:
            print('ex', e)
    
    # Se ver pista amarela, grava o ponto médio dela como alvo no objeto. Se não encontrar, retorna ()
    def encontrar_pista(self):
        frame_hsv = cv2.cvtColor(self.frame_original, cv2.COLOR_BGR2HSV) # No OpenCV, o canal H vai de 0 até 179,

        # Cor da faixa amarela da pista
        cor_inicial, cor_final = np.array([26, 250, 250]), np.array([32, 255, 255])
        segmentado_cor = cv2.inRange(frame_hsv, cor_inicial, cor_final)

        # A operação MORPH_CLOSE fecha todos os buracos na máscara menores  que um quadrado 7x7. 
        segmentado_cor = cv2.morphologyEx(segmentado_cor,cv2.MORPH_CLOSE,np.ones((7, 7)))
        
        ### MUITO MELHOR QUE AQUELAS 300 LINHAS COM ARUCO
        # Remove as n colunas mais à direita da imagem, isso faz nas bifurcações
        # ele sempre pegar o caminho mais à esquerda
        n = 170
        # Tira os n pixels da direita
        for i in range(len(segmentado_cor)):
            segmentado_cor[i][-n:] = [0]*n
        
        media = center_of_mass(segmentado_cor)

        # Desenhamos os contornos encontrados no frame
        contornos, arvore = cv2.findContours(segmentado_cor.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) 
        cv2.drawContours(self.frame_modificado, contornos, -1, [0, 0, 255], 5)
        
        self.debug = segmentado_cor
        
        # Grava o ponto que vai ser perseguido 
        self.ponto_alvo = media
        self.ponto_pista = media

    # Retorna 0 se não tiver creeper na tela ou ele tiver muito longe. Se não, retorna 1 e armazena o CM do creeper como alvo
    # Se ver um creeper (com área o suficente pra parecer perto) grava o ponto médio dele como alvo
    def encontrar_creeper(self):
        # No OpenCV, o canal H vai de 0 até 179, logo cores similares ao vermelho puro (H=0) estão entre H=-8 e H=8. 
        frame_hsv = cv2.cvtColor(self.frame_original, cv2.COLOR_BGR2HSV)
 
        if self.objetivo[0] == 'blue':
            cor_inicial, cor_final = np.array([90, 140, 250]), np.array([100, 180, 255]) 
            segmentado_cor = cv2.inRange(frame_hsv, cor_inicial, cor_final)

        if self.objetivo[0] == 'red':
            cor_inicial, cor_final = np.array([175, 200,200]), np.array([180, 255,255]) 
            segmentado_cor = cv2.inRange(frame_hsv, cor_inicial, cor_final)

            cor_inicial2, cor_final2 = np.array([0, 200,200]), np.array([5, 255,255]) 
            segmentado_cor += cv2.inRange(frame_hsv, cor_inicial2, cor_final2)

        if self.objetivo[0] == 'green':
            cor_inicial, cor_final = np.array([55, 50,50]), np.array([60, 255,255])
            segmentado_cor = cv2.inRange(frame_hsv, cor_inicial, cor_final)

        segmentado_cor = cv2.morphologyEx(segmentado_cor,cv2.MORPH_CLOSE,np.ones((7, 7)))

        # Se não for mostrar segmentado da pista mostra o do creeper
        self.debug = segmentado_cor

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
        if maior_contorno_area > area_min_creeper:
            self.ponto_creeper = media
            self.ponto_alvo = media
        else:
            self.ponto_creeper = ()        
        
    # Função para alterar a velocidade vel_lin e vel_ang são listas de 3 elementos
    def veloc(self, vel_lin, vel_ang):
        linX, linY, linZ = vel_lin
        angX, angY, angZ = vel_ang
        self.velocidade = Twist(Vector3(linX, linY, linZ), Vector3(angX, angY, angZ))

    # Função para perseguir o ponto armazenado  
    def perseguir_ponto(self):
        self.veloc([0,0,0],[0,0,0])

        # Distância mínima até o objeto à frente (com 10 graus pra cada lado)
        distancia = np.min(self.scan[:10]+self.scan[-10:])
        vml = 0.3 # Velocidade maxima linear
        vma = 0.3 # velocidade maxima angular

        if len(self.ponto_alvo) != 0:
            # Quanto mais fora do centro está o alvo mais rápido ele vira e mais devagar ele se move
            dif = abs(self.ponto_alvo[0] - self.centro[0]) 
            p = dif/320 # delta/total
            if p == 1.0: # se não ver o alvo fica girando
                self.veloc([0,0,0],[0,0,0.3])
            
            # Se a camera tiver alinhada com o alvo e não estiver muito proximo
            if (self.ponto_alvo[0] >= self.centro[0]):
                self.veloc([vml*(1-p),0,0],[0,0,-vma*p]) # girar para esquerda do robô (sentido anti-horário)
            if (self.ponto_alvo[0] < self.centro[0]):
                self.veloc([vml*(1-p),0,0],[0,0,vma*p]) # girar para direita do robô (sentido horário)

        # Publicar (Executar) velocidade armazenada no no robô 
        self.pub_velocidade.publish(self.velocidade)
        rospy.sleep(0.1)
            
    # Função executada ao final de cada iteração do while
    def loop(self):

        if len(self.scan) != 0:
            x,y,ang = self.posicao
            distancia = self.distancia
            print('')
            print("Posição (x,y): ({:.2f} , {:.2f})   | α: {:.1f}º".format(x, y, ang))
            print('Scan de proximidade: ', distancia)

            # Persegue creeper
            if len(self.ponto_creeper)!= 0 and self.sucesso == 0:
                self.ponto_alvo = self.ponto_creeper

                # Se tiver muito proximo do creeper diz que cumpriu a missão
                if distancia < 0.3:
                    self.sucesso = 1
                    self.ponto_alvo = self.ponto_pista
                    print("\n\nCheguei no Creeper!")
                    print("Voltando a perseguir a pista...")
                    # Girar
                    vel = Twist(Vector3(0,0,0), Vector3(0,0,math.pi/6))    
                    for i in range(3):
                        self.pub_velocidade.publish(vel)
                        rospy.sleep(1)

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