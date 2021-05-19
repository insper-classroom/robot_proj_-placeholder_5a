#! /usr/bin/env python3
# -*- coding:utf-8 -*-

from __future__ import print_function, division
import rospy
import numpy as np
import math
import cv2
import cv2.aruco as aruco
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import LaserScan
import tf2_ros
from geometry_msgs.msg import Twist, Vector3, Pose, Vector3Stamped

from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from math import pi

print("EXECUTE ANTES da 1.a vez: ")
print("wget https://github.com/Insper/robot21.1/raw/main/projeto/ros_projeto/scripts/MobileNetSSD_deploy.caffemodel")
print("PARA TER OS PESOS DA REDE NEURAL")

import visao_module

######################################## MISSÕES
goal1 = ("blue", 12, "dog")
goal2 = ("green", 23, "horse")
goal3 = ("red", 11, "cow")
########################################

bridge = CvBridge()

cv_image = None
centro = [320, 240]
media = [640, 240]
ESTADO = 1 
atraso = 1.5E9 # 1 segundo e meio. Em nanossegundos
ranges = None
minv = 0
maxv = 10
distancia = 20000
id = 8000
area_creeper = -20
cm_creeper = [640,240]

area = 0.0 # Variavel com a area do maior contorno

SEGUIR_PISTA_APENAS = False  #NAO PROCURA CREEPER

# Só usar se os relógios ROS da Raspberry e do Linux desktop estiverem sincronizados. 
# Descarta imagens que chegam atrasadas demais
check_delay = False 

resultados = [] # Criacao de uma variavel global para guardar os resultados vistos

x = 0
y = 0
z = 0 


frame = "camera_link"
# frame = "head_camera"  # DESCOMENTE para usar com webcam USB via roslaunch tag_tracking usbcam

tfl = 0

tf_buffer = tf2_ros.Buffer()

def distancia_euclidiana(x_bif,y_bif):
    global x
    global y
    dist = math.sqrt((x-x_bif)**2+(y-y_bif)**2)
    return dist

def scaneou(dado):
    global ranges
    global minv
    global maxv
    global distancia
    #("Faixa valida: ", dado.range_min , " - ", dado.range_max )
    #print("Leituras:")
    ranges = np.array(dado.ranges).round(decimals=2)
    minv = dado.range_min 
    maxv = dado.range_max
    distancia = ranges[0]

def odometria(data):
    global x
    global y
    x = data.pose.pose.position.x
    y = data.pose.pose.position.y
    #print("x:",x)
    #print("y:",y)

# A função a seguir é chamada sempre que chega um novo frame
def roda_todo_frame(imagem):
    #print("frame")
    global ESTADO
    global cv_image
    global media
    global centro
    global resultados
    global id
    global area_creeper
    global cm_creeper

    now = rospy.get_rostime()
    imgtime = imagem.header.stamp
    lag = now-imgtime # calcula o lag
    delay = lag.nsecs
    # print("delay ", "{:.3f}".format(delay/1.0E9))
 
  

    if delay > atraso and check_delay==True:
        # Esta logica do delay so' precisa ser usada com robo real e rede wifi 
        # serve para descartar imagens antigas
        print("Descartando por causa do delay do frame:", delay)
        return 
    try:
        temp_image = bridge.compressed_imgmsg_to_cv2(imagem, "bgr8")
        # Note que os resultados já são guardados automaticamente na variável
        # chamada resultados
    #    centro, saida_net, resultados =  visao_module.processa(temp_image)    ##########################################      NECESSARIO APENAS PARA CONCEITO A
    #    for r in resultados:
            # print(r) - print feito para documentar e entender
            # o resultado            
    #        pass

        # Desnecessário - Hough e MobileNet já abrem janelas
        

        cv_image = temp_image.copy()#saida_net.copy()

        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        aruco_dict  = aruco.getPredefinedDictionary(aruco.DICT_6X6_250) ######################################
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict) #, parameters=parameters)

        if ids is not None:
            id = ids[0]
            #for i in range(len(ids)):
             #   print('ID: {}'.format(ids[i]))
            
            #for c in corners[i]: 
             #   for canto in c:
              #      print("Corner {}".format(canto))

        aruco.drawDetectedMarkers(cv_image, corners, ids)          

        if SEGUIR_PISTA_APENAS == False:
            cm_creeper, img_segmentada_creeper, area_creeper = visao_module.identifica_cor_creeper(cv_image, goal1[0])   #ALTERAR GOAL PARA DETERMINADO CREEPER

        media, img_segmentada_pista = visao_module.identifica_cor_pista(cv_image)

        #cv2.imshow("cv_image_segmentada_creeper",img_segmentada_creeper)
        #cv2.imshow("cv_image_segmentada_pista",img_segmentada_pista)
        cv2.imshow("cv_image",cv_image)
        cv2.waitKey(1)
    except CvBridgeError as e:
        print('ex', e)
    
if __name__=="__main__":
    rospy.init_node("cor")

    topico_imagem = "/camera/image/compressed"

    recebedor = rospy.Subscriber(topico_imagem, CompressedImage, roda_todo_frame, queue_size=4, buff_size = 2**24)
    recebe_scan = rospy.Subscriber("/scan", LaserScan, scaneou)
    odom_sub = rospy.Subscriber('/odom', Odometry, odometria)

    print("Usando ", topico_imagem)

    velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)

    tfl = tf2_ros.TransformListener(tf_buffer) #conversao do sistema de coordenadas 
    tolerancia = 25
    

    x_primeira_bif, y_primeira_bif = 2000,2000
    x_segunda_bif, y_segunda_bif = 2000, 2000
    # GIRA 180 GRAUS
    # vel = Twist(Vector3(0,0,0), Vector3(0,0,pi/5))    
    # for i in range(6):
    #     velocidade_saida.publish(vel)
    #     rospy.sleep(1)
    vira = True
    tem_creeper = False
    #ESTADO = 3   #VIRA 180 NO INICIO DO PERCURSO
    try:
        # Inicializando - por default gira no sentido anti-horário
        while not rospy.is_shutdown():
            
            #print(area_creeper)
            if area_creeper > 1350 and tem_creeper == False:
                vel = Twist(Vector3(0.1,0,0), Vector3(0,0,0.05))
                velocidade_saida.publish(vel)
                vel = Twist(Vector3(0.05,0,0), Vector3(0,0,0))
                velocidade_saida.publish(vel)
                vel = Twist(Vector3(0.0,0,0), Vector3(0,0,0))
                velocidade_saida.publish(vel)
                ESTADO = 5

            print('Estado:',ESTADO)
            #print(distancia)
            if ESTADO == 1 :                                            #segue linha amarela
                #print("centro_massa:",  media[0])
                #print("centro_imagem:", centro[0])
                if media[0] > centro[0]:
                    #print("direita")
                    vel = Twist(Vector3(0.2,0,0), Vector3(0,0,-0.15))
                if media[0] < centro[0]:
                    #print("esquerda")
                    vel = Twist(Vector3(0.2,0,0), Vector3(0,0,0.15))
                velocidade_saida.publish(vel)
            
                if distancia < 1 and id == 100:
                    vel = Twist(Vector3(0.1,0,0), Vector3(0,0,0))
                    velocidade_saida.publish(vel)
                    vel = Twist(Vector3(0.05,0,0), Vector3(0,0,0))
                    velocidade_saida.publish(vel)
                    vel = Twist(Vector3(0.0,0,0), Vector3(0,0,0))
                    velocidade_saida.publish(vel)
                    ESTADO = 2

                if distancia < 0.5 and id == 150:
                    vel = Twist(Vector3(0.1,0,0), Vector3(0,0,0))
                    velocidade_saida.publish(vel)
                    vel = Twist(Vector3(0.05,0,0), Vector3(0,0,0))
                    velocidade_saida.publish(vel)
                    vel = Twist(Vector3(0.0,0,0), Vector3(0,0,0))
                    velocidade_saida.publish(vel)
                    vira = True
                    ESTADO = 3

                if distancia < 0.5 and id == 50:
                    vel = Twist(Vector3(0.1,0,0), Vector3(0,0,0))
                    velocidade_saida.publish(vel)
                    vel = Twist(Vector3(0.05,0,0), Vector3(0,0,0))
                    velocidade_saida.publish(vel)
                    vel = Twist(Vector3(0.0,0,0), Vector3(0,0,0))
                    velocidade_saida.publish(vel)
                    vira = False
                    ESTADO = 3 

                if distancia < 0.7 and id ==200:
                    vel = Twist(Vector3(0.1,0,0), Vector3(0,0,0))
                    velocidade_saida.publish(vel)
                    vel = Twist(Vector3(0.05,0,0), Vector3(0,0,0))
                    velocidade_saida.publish(vel)
                    vel = Twist(Vector3(0.0,0,0), Vector3(0,0,0))
                    velocidade_saida.publish(vel)
                    ESTADO = 4

                if distancia_euclidiana(x_primeira_bif, y_primeira_bif) < 0.2 and vira == True:
                    vel = Twist(Vector3(0.1,0,0), Vector3(0,0,0))
                    velocidade_saida.publish(vel)
                    vel = Twist(Vector3(0.05,0,0), Vector3(0,0,0))
                    velocidade_saida.publish(vel)
                    vel = Twist(Vector3(0.0,0,0), Vector3(0,0,0))
                    velocidade_saida.publish(vel)
                    ESTADO = 2 

                if distancia_euclidiana(x_segunda_bif, y_segunda_bif) < 0.2:
                    vel = Twist(Vector3(0.1,0,0), Vector3(0,0,0.05))
                    velocidade_saida.publish(vel)
                    vel = Twist(Vector3(0.05,0,0), Vector3(0,0,0))
                    velocidade_saida.publish(vel)
                    vel = Twist(Vector3(0.0,0,0), Vector3(0,0,0))
                    velocidade_saida.publish(vel)
                    ESTADO = 4


            if ESTADO == 2:                                          #pega o caminho da esquerda na primeira bifurcacao(gira 45 graus)
                vel = Twist(Vector3(0,0,0), Vector3(0,0,pi/20))
                for i in range(5):
                    velocidade_saida.publish(vel)
                    rospy.sleep(1)
                x_primeira_bif = x
                y_primeira_bif = y

                vel = Twist(Vector3(0.2,0,0), Vector3(0,0,0))
                for i in range(2):
                    velocidade_saida.publish(vel)
                    rospy.sleep(1)

                ESTADO = 1
            
            if ESTADO == 3:                                             #vira 180 graus depois de chegar em beco sem saida ou tocar no creeper
                vel = Twist(Vector3(0,0,0), Vector3(0,0,pi/5))    
                for i in range(5):
                    velocidade_saida.publish(vel)
                    rospy.sleep(1)
                ESTADO = 1
            
            
            if ESTADO == 4:
                vel = Twist(Vector3(0,0,0), Vector3(0,0,pi/10))         #pega o caminho da esquerda na bifurcacao do balao (gira 90 graus)
                for i in range(5):
                    velocidade_saida.publish(vel)
                    rospy.sleep(1)
                x_segunda_bif = x
                y_segunda_bif = y

                vel = Twist(Vector3(0.2,0,0), Vector3(0,0,0))
                for i in range(2):
                    velocidade_saida.publish(vel)
                    rospy.sleep(1)

                ESTADO = 1


            if ESTADO == 5:                                             #Gira ate centralizar no creeper
                if cm_creeper[0] > centro[0]:
                    #print("direita")
                    vel = Twist(Vector3(0,0,0), Vector3(0,0,-0.2))
                if cm_creeper[0] < centro[0]:
                    #print("esquerda")
                    vel = Twist(Vector3(0,0,0), Vector3(0,0,0.2))
                velocidade_saida.publish(vel)

                if abs(cm_creeper[0] - centro[0]) < 10 :
                    tem_creeper = True
                    ESTADO = 6


            if ESTADO == 6:                                               #vai em direção ao creeper
                if cm_creeper[0] > centro[0]:
                    #print("direita")
                    vel = Twist(Vector3(0.2,0,0), Vector3(0,0,-0.15))
                if cm_creeper[0] < centro[0]:
                    #print("esquerda")
                    vel = Twist(Vector3(0.2,0,0), Vector3(0,0,0.15))
                velocidade_saida.publish(vel)
                #print(distancia)

                if distancia < 0.2:
                    ESTADO = 3 
    

    except rospy.ROSInterruptException:
        print("Ocorreu uma exceção com o rospy")



