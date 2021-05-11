#! /usr/bin/env python
# -*- coding:utf-8 -*-

from __future__ import division, print_function


import rospy
import numpy as np
import tf
import math
import cv2
import time
from geometry_msgs.msg import Twist, Vector3, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import mobilenet_simples as mnet



def processa(frame):
    '''Use esta funcao para basear o processamento do seu robo'''

    result_frame, result_tuples = mnet.detect(frame)

    centro = (frame.shape[1]//2, frame.shape[0]//2)


    def cross(img_rgb, point, color, width,length):
        cv2.line(img_rgb, (point[0] - int(length/2), point[1]),  (point[0] + int(length/2), point[1]), color ,width, length)
        cv2.line(img_rgb, (point[0], point[1] - int(length/2)), (point[0], point[1] + int(length/2)),color ,width, length)

    cross(result_frame, centro, [255,0,0], 1, 17)


    return centro, result_frame, result_tuples

def center_of_mass(mask):
    """ Retorna uma tupla (cx, cy) que desenha o centro do contorno"""
    M = cv2.moments(mask)
    # Usando a expressão do centróide definida em: https://en.wikipedia.org/wiki/Image_moment
    if M["m00"] == 0:
        M["m00"] = 1
    cX = int(M["m10"] / M["m00"])
    cY = int(M["m01"] / M["m00"])
    return [int(cX), int(cY)]

def crosshair(img, point, size, color):
    """ Desenha um crosshair centrado no point.
        point deve ser uma tupla (x,y)
        color é uma tupla R,G,B uint8
    """
    x,y = point
    cv2.line(img,(x - size,y),(x + size,y),color,5)
    cv2.line(img,(x,y - size),(x, y + size),color,5)


def identifica_cor(frame):
    '''
    Segmenta o maior objeto cuja cor é parecida com cor_h (HUE da cor, no espaço HSV).
    '''

    # No OpenCV, o canal H vai de 0 até 179, logo cores similares ao
    # vermelho puro (H=0) estão entre H=-8 e H=8.
    # Precisamos dividir o inRange em duas partes para fazer a detecção
    # do vermelho:
    bgr = frame.copy()

    frame_hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)

    cor_menor = np.array([25, 50,50])
    cor_maior = np.array([32, 255,255])
    segmentado_cor = cv2.inRange(frame_hsv, cor_menor, cor_maior)

   # cor_menor = np.array([105, 50,50])
    #cor_maior = np.array([125, 255,255])
    #segmentado_cor += cv2.inRange(frame_hsv, cor_menor, cor_maior)

    # Note que a notacão do numpy encara as imagens como matriz, portanto o enderecamento é
    # linha, coluna ou (y,x)
    # Por isso na hora de montar a tupla com o centro precisamos inverter, porque
    centro = (int(frame.shape[1]//2), int(frame.shape[0]//2))


    def cross(img_rgb, point, color, width,length):
        cv2.line(img_rgb, (int(point[0] - length/2), int(point[1])),  (int(point[0] + length/2), int(point[1])), color ,width, length)
        cv2.line(img_rgb, (int(point[0]), int(point[1] - length/2)), (int(point[0]), int(point[1] + length/2)),color ,width, length)



    # A operação MORPH_CLOSE fecha todos os buracos na máscara menores
    # que um quadrado 7x7. É muito útil para juntar vários
    # pequenos contornos muito próximos em um só.
    segmentado_cor = cv2.morphologyEx(segmentado_cor,cv2.MORPH_CLOSE,np.ones((7, 7)))

    segmentado_cor_bgr = cv2.cvtColor(segmentado_cor, cv2.COLOR_GRAY2BGR)

    cm = center_of_mass(segmentado_cor)

    crosshair(segmentado_cor_bgr, centro, 25, color=(0,255,0))


    crosshair(segmentado_cor_bgr, cm, 15, color=(0,0,255))


    return cm, segmentado_cor_bgr#, result_tuples
