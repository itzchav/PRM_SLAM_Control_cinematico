#!/usr/bin/python3

import subprocess
import sys
from math import sin, cos, pi, sqrt
import math
import rospy
import cv2
import subprocess

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import os
import networkx as nx

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import tf
from nav_msgs.msg import OccupancyGrid

from scipy.spatial import KDTree
from heapq import heappush, heappop
import time

import random
import sys, select, os
if os.name == 'nt':
  import msvcrt
else:
  import tty, termios


from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray


V_max = 0.22; W_max = 2.84; 
num=1147



def publish_markers(self):
        # Publish visual data in Rviz
        markerArray = MarkerArray()
        marker = Marker()
        marker.header.frame_id = "odom"
        marker.type = marker.CYLINDER
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.01
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = self.punto_desado_xh
        marker.pose.position.y = self.punto_desado_yh
        marker.pose.position.z = 0
        
        markerArray.markers.append(marker)
        self.publisher.publish(markerArray)


def verificar_region_blanca(imagen_binaria, x_pixeles, y_pixeles):
    if 0 <= x_pixeles < imagen_binaria.shape[1] and 0 <= y_pixeles < imagen_binaria.shape[0]:
        if imagen_binaria[int(y_pixeles), int(x_pixeles)] == 255:
            imagen_binaria[int(y_pixeles), int(x_pixeles)] == 0
            return True  # El punto esta en una region blanca
    return False  # El punto esta fuera de la imagen o en una region negra

def metros_a_pixeles(self, x, y):  
    pixel_x=self.pixel_origen_x+round(x/self.resx)
    pixel_y=self.pixel_origen_y-round(y/self.resy)
    return pixel_x, pixel_y


def pixeles_a_metros(self, pixel_x, pixel_y):
    y=(-self.origen_y+self.y_max+self.origen_global_y)
    metros_x =  self.origen_global_x + (pixel_x) * self.resx 
    metros_y =  y-(pixel_y) * self.resy     
    return metros_x, metros_y


def generar_puntos_aleatorios_radio(self, imagen_binaria, imagen_color):
    pixeles_aleatorios = []
    self.distancia_minima = 40
    self.distancia_min_puntos_ruta = 10
    plo = None
    distancia_plo = None
    
    self.puntos_guardados_pixeles= [metros_a_pixeles(self, px, py) for px, py in self.centros]
    self.puntos_ruta_pixeles= [metros_a_pixeles(self, px, py) for px, py in self.ruta]

    for (px, py) in self.puntos_guardados_pixeles:
        imagen_color[py, px] = [255, 105, 180]

    for _ in range(self.nodos):
        while True:
            rand_x = random.randint(0, imagen_binaria.shape[1] - 1)
            rand_y = random.randint(0, imagen_binaria.shape[0] - 1)
            # Verifica si el pixel en las coordenadas es blanco
            if imagen_binaria[rand_y, rand_x] == 255:
                # Calcula la distancia mínima con todos los puntos guardados
                distancias = [distancia_entre_puntos((rand_x, rand_y), (y,x)) for y,x in self.puntos_guardados_pixeles]
                distancia_2= [distancia_entre_puntos((rand_x, rand_y), (y,x)) for y,x in self.puntos_ruta_pixeles]
                # Verifica si la distancia mínima es al menos 40 píxeles de cada punto guardado
                if all(distancia >= self.distancia_minima for distancia in distancias) and all(distancia >= self.distancia_min_puntos_ruta for distancia in distancia_2):
                    # El punto es válido
                    imagen_color[rand_y, rand_x] = [0, 100, 255]  # Amarillo
                    pixeles_aleatorios.append((rand_x, rand_y))
                    x_metros, y_metros = pixeles_a_metros(self, rand_x, rand_y)
                    distancia = distancia_entre_puntos((x_metros, y_metros), (self.inicio_x, self.inicio_y))
                    
                    if plo is None or distancia > distancia_plo:
                        plo = (rand_x, rand_y)
                        distancia_plo = distancia
                    break

    if plo is not None:
        imagen_color[plo[1], plo[0]] = [0, 255, 0]  # Verde

    return plo

def generar_puntos_aleatorios(self, imagen_binaria, imagen_color):
    pixeles_aleatorios = []
    plo = None
    distancia_plo = None
    
    

    for _ in range(self.nodos):
        while True:
            # Genera coordenadas aleatorias dentro de la imagen

            rand_x = random.randint(0, imagen_binaria.shape[1] - 1)
            rand_y = random.randint(0, imagen_binaria.shape[0] - 1)

            # Verifica si el pixel en las coordenadas es blanco (puedes ajustar el criterio de blanco segun tu necesidad)
            if imagen_binaria[rand_y, rand_x] == 255:
                # Pone un punto morado en las coordenadas generadas
                imagen_color[rand_y, rand_x] = [255, 0, 255]  # Morado
                pixeles_aleatorios.append((rand_x, rand_y))
                
                # Convierte las coordenadas del pixel al sistema de metros
                x_metros, y_metros = pixeles_a_metros(self,rand_x, rand_y)

                # Calcula la distancia entre el punto generado y el objetivo (3, 5)
                distancia = distancia_entre_puntos((x_metros, y_metros), (self.inicio_x, self.inicio_y))
                if plo is None or distancia > distancia_plo:
                        plo = (rand_x, rand_y)
                        distancia_plo = distancia
                break

    if plo is not None:
        imagen_color[plo[1], plo[0]] = [0, 255, 0]  # Verde

    return plo


def generar_puntos_aleatorios_al_no_encotrar_ruta(self, imagen_binaria, imagen_color):
    pixeles_aleatorios = []
    distancia_pcm_na = None
    self.distancia_minima = 40
    self.distancia_min_puntos_ruta = 10
    pcm_na = None#Punto cercano a la meta no alcanzable

    self.puntos_guardados_pixeles= [metros_a_pixeles(self, px, py) for px, py in self.centros]
    self.puntos_ruta_pixeles= [metros_a_pixeles(self, px, py) for px, py in self.ruta]



    for (px, py) in self.puntos_guardados_pixeles:
        imagen_color[py, px] = [255, 105, 180]

    for (px, py) in self.meta:
        imagen_color[py, px] = [255, 0, 0]
    for _ in range(self.nodos):
        while True:
            rand_x = random.randint(0, imagen_binaria.shape[1] - 1)
            rand_y = random.randint(0, imagen_binaria.shape[0] - 1)
            # Verifica si el pixel en las coordenadas es blanco
            if imagen_binaria[rand_y, rand_x] == 255:
                # Calcula la distancia mínima con todos los puntos guardados
                distancias_meta = [distancia_entre_puntos((rand_x, rand_y), (y,x)) for y,x in self.meta]
                distancias = [distancia_entre_puntos((rand_x, rand_y), (y,x)) for y,x in self.puntos_guardados_pixeles]
                distancia_2= [distancia_entre_puntos((rand_x, rand_y), (y,x)) for y,x in self.puntos_ruta_pixeles]
                # Verifica si la distancia mínima es al menos 40 píxeles de cada punto guardado
                if all(distancia >= self.distancia_minima for distancia in distancias) and all(distancia >= self.distancia_min_puntos_ruta for distancia in distancia_2) and all(distancia >= self.distancia_minima for distancia in distancias_meta):
                
                
                    imagen_color[rand_y, rand_x] = [0, 100, 0]  # Amarillo
                    pixeles_aleatorios.append((rand_x, rand_y))
                    x_metros, y_metros = pixeles_a_metros(self, rand_x, rand_y)
                    x_meta, y_meta= pixeles_a_metros(self, self.meta[0][0], self.meta[0][1])
                    distancia = distancia_entre_puntos((x_metros, y_metros), (x_meta,y_meta))
                    
                    if pcm_na is None or distancia < distancia_pcm_na:
                        pcm_na = (rand_x, rand_y)
                        distancia_pcm_na = distancia
                    break

    if pcm_na is not None:
        imagen_color[pcm_na[1], pcm_na[0]] = [0, 255, 0]  # Verde
        #cv2.imshow("Altura", imagen_color)
        #cv2.waitKey(0)
        #cv2.destroyAllWindows()

    return pcm_na


    



def distancia_entre_puntos(p1, p2):
    return np.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)


def hay_obstaculo_entre_puntos(punto1, punto2, imagen_binaria):
    # Verificar si hay obstáculos entre dos puntos utilizando la línea entre ellos
    linea = np.linspace(punto1, punto2, num=10).astype(int)
    for punto in linea:
        if imagen_binaria[punto[0], punto[1]] == 0:
            return True
    return False

def generar_muestras_validas(imagen_binaria, numero_de_muestras):
    # Obtener las coordenadas de los puntos blancos en la imagen binaria
    puntos_validos = np.column_stack(np.where(imagen_binaria == 255))  # Puntos blancos
    
    # Seleccionar aleatoriamente un número específico de muestras
    indices_muestras = np.random.choice(len(puntos_validos), numero_de_muestras, replace=False)
    muestras = puntos_validos[indices_muestras]
    
    # Convertir la imagen binaria a una imagen en color (BGR)
    imagen_color = cv2.cvtColor(imagen_binaria, cv2.COLOR_GRAY2BGR)
    
    # Marcar los puntos de muestra en amarillo (255, 255, 0) en la imagen en color
    for (y, x) in muestras:
        imagen_color[y, x] = [0, 255, 255]  # Color amarillo en formato BGR
        
    return muestras


def shortest(graph, start_pixel, goal_pixel):
    start_node = tuple(start_pixel)
    goal_node = tuple(goal_pixel)
    
    distancia = {node: float('inf') for node in graph.nodes}
    parent = {node: None for node in graph.nodes}
    
    distancia[start_node] = 0
    
    queue = [(0, start_node)]
    
    while queue:
        current_distance, current_node = min(queue, key=lambda x: x[0])
        queue.remove((current_distance, current_node))
        
        for neighbor in graph.neighbors(current_node):
            if distancia[current_node] + 1 < distancia[neighbor]:
                distancia[neighbor] = distancia[current_node] + 1
                parent[neighbor] = current_node
                queue.append((distancia[neighbor], neighbor))
    
    if distancia[goal_node] == float('inf'):
        return float('inf'), None
    camino = obten_camino(parent, goal_node)

    return camino

def obten_camino(parent, final):
    camino = []
    while final:
        camino.append(final)
        final = parent[final]
    return camino[::-1]

def get_occupancy(image, pos):
    # Verificar si el píxel está dentro de los límites de la imagen
    if 0 <= pos[1] < image.shape[0] and 0 <= pos[0] < image.shape[1]:
        # Obtener el valor del píxel en la posición dada
        pixel_value = image[pos[1], pos[0]]
        # Devuelve 0 si el píxel es blanco (espacio libre), 1 si no lo es.
        return 0 if np.all(pixel_value == 255) else 1
    return 1


def dibujar_grafo(imagen_color, grafo, puntos):
    # Copiar la imagen original para dibujar sobre ella
    imagen_dibujo = imagen_color.copy()
    # Dibujar nodos
    for punto in puntos:
        cv2.circle(imagen_dibujo, (int(punto[1]), int(punto[0])), 5, (0, 255, 255), -1)  # Amarillo
    # Dibujar aristas
    for (p1, p2, data) in grafo.edges(data=True):
        cv2.line(imagen_dibujo, (int(p1[1]), int(p1[0])), (int(p2[1]), int(p2[0])), (255, 0, 0), 1)  # Rojo
    return imagen_dibujo
"""
def encontrar_pixel_blanco_mas_cercano(image, x, y):
    # Verificar si el píxel está en una zona blanca
    if image[x,y] == 255:
        return x, y#, True    
    radio = 2
    while True:
        for i in range(-radio, radio + 1):
            for j in range(-radio, radio + 1):
                x_vecino = x + i
                y_vecino = y + j
                if 0 <= x_vecino < image.shape[0] and 0 <= y_vecino < image.shape[1]:
                    if image[x_vecino, y_vecino] == 255:
                        # Mostrar el punto blanco encontrado en la imagen
                        image[x_vecino,y_vecino] = 180                       
                        return x_vecino, y_vecino#, True   
        radio += 1
        # Si no se encontró un píxel blanco en el radio actual, continuar con un mayor radio de búsqueda.
        if radio > max(image.shape):
            break
    # Si no se encontró ningún píxel blanco cercano, devolver el punto original y bandera falsa.
    return x, y#, False

"""
def encontrar_pixel_blanco_mas_cercano(image, x, y):
    # Verificar si el píxel está en una zona blanca
    if image[x, y] == 255:
        return x, y
    
    # Inicializar el radio de búsqueda
    radio = 1
    max_radio = max(image.shape[0], image.shape[1])
    
    while radio <= max_radio:
        for i in range(-radio, radio + 1):
            for j in range(-radio, radio + 1):
                x_vecino = x + i
                y_vecino = y + j
                
                # Verificar que el vecino está dentro de los límites de la imagen
                if 0 <= x_vecino < image.shape[0] and 0 <= y_vecino < image.shape[1]:
                    if image[x_vecino, y_vecino] == 255:
                        # Mostrar el punto blanco encontrado en la imagen (opcional)
                        image[x_vecino, y_vecino] = 180
                        return x_vecino, y_vecino
        
        # Aumentar el radio de búsqueda
        radio += 1
    
    # Si no se encontró ningún píxel blanco, devolver el punto original
    return x, y




def get_white_points(self,image,random=False):
    """
    Encuentra los puntos blancos en la imagen basándose en el valor del contador.
    
    - Contador = 1: Encuentra el primer y último punto blanco en el eje X.
    - Contador = 2: Encuentra el primer punto blanco en el borde izquierdo y el primer punto blanco en el borde derecho.
    - Contador = 3: Encuentra el primer punto blanco en el borde inferior y el primer punto blanco en el borde izquierdo.
    - Contador = 4: Encuentra el primer y último punto blanco en el eje Y.
    - Contador = 5: Encuentra el primer punto blanco en el borde superior y el primer punto blanco en el borde inferior.
    - Contador = 6: Encuentra el primer punto blanco en el borde superior y el primer punto blanco en el borde derecho.
    - Contador = 7: Encuentra el primer punto blanco en el borde inferior y el primer punto blanco en el borde derecho.
    - Contador = 8: Encuentra el primer punto blanco en el borde inferior y el último punto blanco en el borde izquierdo.
    - Contador = 9: Encuentra el último punto blanco en el borde superior y el primer punto blanco en el borde izquierdo.
    """
    coords = np.column_stack(np.where(image == 255))
    if random:
        contador = np.random.randint(1, 10)  # Selecciona un número aleatorio entre 1 y 9
    else:
        contador = self.ultimo

    if contador == 1:
        # Encuentra el primer y último punto blanco en el eje X
        coords_sorted_by_x = coords[coords[:, 1].argsort()]
        p1 = coords_sorted_by_x[0]  # Primer pixel blanco desde la izquierda
        p2 = coords_sorted_by_x[-1] # Último pixel blanco desde la derecha
        return p1, p2
    
    elif contador == 2:
        # Encuentra el primer punto blanco en el borde izquierdo y el primer punto blanco en el borde derecho
        left_edge = np.min(coords[:, 1])
        right_edge = np.max(coords[:, 1])
        p1 = coords[coords[:, 1] == left_edge][0]  # Primer blanco en el borde izquierdo
        p2 = coords[coords[:, 1] == right_edge][0] # Primer blanco en el borde derecho
        return p1, p2
    
    elif contador == 3:
        # Encuentra el primer pixel blanco en el borde inferior y el primer pixel blanco en el borde izquierdo
        bottom_edge = np.max(coords[:, 0])
        left_edge = np.min(coords[:, 1])
        p1 = coords[coords[:, 0] == bottom_edge][0]  # Primer blanco en el borde inferior
        p2 = coords[coords[:, 1] == left_edge][0]    # Primer blanco en el borde izquierdo
        return p1, p2
    
    elif contador == 4:
        # Encuentra el primer y último punto blanco en el eje Y
        coords_sorted_by_y = coords[coords[:, 0].argsort()]
        p1 = coords_sorted_by_y[0]  # Primer pixel blanco desde arriba
        p2 = coords_sorted_by_y[-1] # Último pixel blanco desde abajo
        return p1, p2
    
    elif contador == 5:
        # Encuentra el primer punto blanco en el borde superior y el primer punto blanco en el borde inferior
        top_edge = np.min(coords[:, 0])
        bottom_edge = np.max(coords[:, 0])
        p1 = coords[coords[:, 0] == top_edge][0]    # Primer blanco en el borde superior
        p2 = coords[coords[:, 0] == bottom_edge][0] # Primer blanco en el borde inferior
        return p1, p2
    
    elif contador == 6:
        # Encuentra el primer punto blanco en el borde superior y el primer punto blanco en el borde derecho
        top_edge = np.min(coords[:, 0])
        right_edge = np.max(coords[:, 1])
        p1 = coords[coords[:, 0] == top_edge][0]    # Primer blanco en el borde superior
        p2 = coords[coords[:, 1] == right_edge][0] # Primer blanco en el borde derecho
        return p1, p2
    
    elif contador == 7:
        # Encuentra el primer punto blanco en el borde inferior y el primer punto blanco en el borde derecho
        bottom_edge = np.max(coords[:, 0])
        right_edge = np.max(coords[:, 1])
        p1 = coords[coords[:, 0] == bottom_edge][0]    # Primer blanco en el borde inferior
        p2 = coords[coords[:, 1] == right_edge][0] # Primer blanco en el borde derecho
        return p1, p2
    
    elif contador == 8:
        # Encuentra el primer punto blanco en el borde inferior y el último punto blanco en el borde izquierdo
        bottom_edge = np.max(coords[:, 0])
        left_edge = np.min(coords[:, 1])
        p1 = coords[coords[:, 0] == bottom_edge][0]    # Primer blanco en el borde inferior
        p2 = coords[coords[:, 1] == left_edge][-1]    # Último blanco en el borde izquierdo
        return p1, p2
    
    elif contador == 9:
        # Encuentra el último punto blanco en el borde superior y el primer punto blanco en el borde izquierdo
        top_edge = np.min(coords[:, 0])
        left_edge = np.min(coords[:, 1])
        p1 = coords[coords[:, 0] == top_edge][-1]    # Último blanco en el borde superior
        p2 = coords[coords[:, 1] == left_edge][0]    # Primer blanco en el borde izquierdo
        
        #self.contador=0
        return p1, p2
    
    else:
        self.ultimo=1
        #raise ValueError("El valor del contador no está soportado. Use un valor entre 1 y 9.")


def get_first_and_last_white_points(image):
    """
    Encuentra el primer y último punto blanco en el eje X.
    """
    coords=np.column_stack(np.where(image == 255))
    coords_sorted_by_y = coords[coords[:, 0].argsort()]
    
    p1 = coords_sorted_by_y[0] # Primer pixel blanco desde arriba
    p2 = coords_sorted_by_y[-1] # Último pixel blanco desde abajo
    return p1, p2


def calculate_bisector(image, p1, p2, p3):
    color_bisector = (0, 255, 0)
    color_points = (0, 0, 255)
    thickness = 2
    length = 100
    """
    Calcula el punto de intersección de la bisectriz desde el punto p2 
    con el segmento que pasa por p1 y p3, y visualiza la bisectriz en la imagen usando OpenCV.
    """
    
    # Convertir las coordenadas de (x, y) a (y, x) para OpenCV
    p1 = (p1[1], p1[0])
    p2 = (p2[1], p2[0])
    p3 = (p3[1], p3[0])
    
    # Crear una copia de la imagen para la visualización
    image_color = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
    
    # Dibujar los puntos en la imagen
    cv2.circle(image_color, p1, 5, color_points, -1)  # Rojo
    cv2.circle(image_color, p2, 5, color_points, -1)  # Rojo
    cv2.circle(image_color, p3, 5, color_points, -1)  # Rojo
    
    # Convertir puntos a arrays de numpy
    P1 = np.array(p1)
    P2 = np.array(p2)
    P3 = np.array(p3)
    
    # Calcular los vectores desde P2
    v1 = P1 - P2
    v2 = P3 - P2
    
    # Normalizar los vectores
    u1 = v1 / np.linalg.norm(v1)
    u2 = v2 / np.linalg.norm(v2)
    
    # Calcular el vector bisector
    u_bis = u1 + u2
    
    # Normalizar el vector bisector
    u_bis = u_bis / np.linalg.norm(u_bis)
    
    # Punto inicial y dirección de la bisectriz
    bisector_start = P2
    bisector_direction = u_bis
    
    # Definir el segmento opuesto
    segment_start = P1
    segment_end = P3
    
    # Calcular la intersección de la bisectriz con el segmento opuesto
    def line_intersection(p1, d1, p2, d2):
        """
        Encuentra la intersección de dos líneas.
        Las líneas están definidas por un punto en la línea y un vector de dirección.
        """
        p1 = np.array(p1)
        d1 = np.array(d1)
        p2 = np.array(p2)
        d2 = np.array(d2)
        
        # Resolver el sistema de ecuaciones
        A = np.array([d1, -d2]).T
        b = p2 - p1
        t = np.linalg.solve(A, b)
        
        # Calcular el punto de intersección
        intersection = p1 + t[0] * d1
        return intersection

    # Definir las direcciones del segmento opuesto
    segment_direction = segment_end - segment_start
    
    # Encontrar el punto de intersección
    intersection_point = line_intersection(bisector_start, bisector_direction, segment_start, segment_direction)
    
    # Dibujar la bisectriz hasta el punto de intersección
    cv2.line(image_color, tuple(P2), tuple(intersection_point.astype(int)), color_bisector, thickness)
    
    # Dibujar el segmento opuesto
    cv2.line(image_color, tuple(P1), tuple(P3), (255, 0, 0), thickness)
    
    return image_color, intersection_point


def dibujar_grafo(imagen_color, grafo, puntos):
    # Copiar la imagen original para dibujar sobre ella
    imagen_dibujo = imagen_color.copy()
    # Dibujar nodos
    for punto in puntos:
        cv2.circle(imagen_dibujo, (int(punto[1]), int(punto[0])), 5, (0, 255, 255), -1)  # Amarillo
    # Dibujar aristas
    for (p1, p2, data) in grafo.edges(data=True):
        cv2.line(imagen_dibujo, (int(p1[1]), int(p1[0])), (int(p2[1]), int(p2[0])), (255, 0, 0), 1)  # Rojo
    return imagen_dibujo

def prm(self, imagen, imagen_reducida, im_bin,  imagen_color, numero_de_muestras, distancia_max, punto_inicio, punto_destino,carpeta):
    path = None
    
    print("generanding")
    self.Velocidad_Lineal=0
    self.Velocidad_Angular=0
    self.velocidad.linear.x = self.Velocidad_Lineal 
    self.velocidad.angular.z = self.Velocidad_Angular
    self.pub.publish(self.velocidad)

    
    img_color = cv2.cvtColor(imagen, cv2.COLOR_GRAY2BGR)
    img_color[punto_inicio] = [255, 0, 0]  # Blue
    img_color[punto_destino] = [0, 0, 255]  # Red
    ruta_completa = os.path.join(carpeta, 'inicio_fin.jpg')
    cv2.imwrite(ruta_completa, img_color)
    

    
    # Bucle para intentar encontrar una ruta
    while path is None:
        print("generating while")
         # Verificar y ajustar el punto de inicio
        x_start, y_start = encontrar_pixel_blanco_mas_cercano(imagen_reducida, punto_inicio[0], punto_inicio[1])
        punto_inicio = (x_start,y_start)

        x_end, y_end = encontrar_pixel_blanco_mas_cercano(imagen_reducida, punto_destino[0], punto_destino[1])
        punto_destino = (x_end,y_end)
        self.plo=punto_destino
        self.plo_x, self.plo_y = pixeles_a_metros(self, self.plo[1], self.plo[0])

        img_color[punto_inicio] = [0, 255, 0]  # Green
        img_color[punto_destino] = [255, 255, 0]  # Cian

        print(punto_inicio, punto_destino)
        print("inicio, fin")
        

        grafo = nx.Graph()
        muestras_validas = generar_muestras_validas(imagen, numero_de_muestras)
        img_color[punto_inicio] = [255, 255, 0]  # Amarillo
        img_color[punto_destino] = [0, 255, 255]  # Cian

        muestras = np.vstack([muestras_validas, punto_inicio, punto_destino])  # Agregar puntos de inicio y destino
        kdtree = KDTree(muestras)
        for i, punto in enumerate(muestras):
            # Encontrar vecinos dentro de la distancia máxima
            distancias, vecinos_indices = kdtree.query(punto, k=25, distance_upper_bound=distancia_max)
            for vecino_indice, distancia in zip(vecinos_indices, distancias):
                if distancia < distancia_max and not hay_obstaculo_entre_puntos(punto, muestras[vecino_indice], im_bin):
                    grafo.add_edge(tuple(punto), tuple(muestras[vecino_indice]), weight=distancia)

        # Dibuja la ruta en la imagen original.
        img_color = cv2.cvtColor(im_bin, cv2.COLOR_GRAY2BGR)
        imagen_con_ruta = img_color.copy()  # Copia la imagen original.
        imagen_con_ruta_2 = img_color.copy()  # Copia la imagen original.

        im=dibujar_grafo(imagen_color, grafo, muestras)
        nombre_imagen = "grafo" + str(self.num_new_img)
        self.num_new_img += 1
        nombre_imagen_con_extension = nombre_imagen + ".jpg"
        ruta_completa = os.path.join(carpeta, nombre_imagen_con_extension)
            

        cv2.imwrite(ruta_completa, im)

        try:
            if nx.has_path(grafo, tuple(punto_inicio), tuple(punto_destino)):
                ruta_optima = nx.shortest_path(grafo, source=tuple(punto_inicio), target=tuple(punto_destino), weight='weight')
                print(len(ruta_optima))

                punto_inicio = pixeles_a_metros(self, punto_inicio[0], punto_inicio[1])
                punto_destino = pixeles_a_metros(self, punto_destino[0], punto_destino[1])
                distancia_total = sqrt((punto_inicio[0] - punto_destino[0])**2 + (punto_inicio[1] - punto_destino[1])**2)

                path = ruta_optima
                for i in range(len(path) - 1):
                    point1 = (path[i][1], path[i][0])  # Intercambiar coordenadas xy a yx
                    point2 = (path[i + 1][1], path[i + 1][0])  # Intercambiar coordenadas xy a yx
                    cv2.line(imagen_con_ruta, point1, point2, (0, 0, 255), 1)  # Dibuja la línea roja entre nodos de la ruta.

                # Dibuja los puntos aleatorios en la imagen
                if self.centros is not None:
                    for punto_guardado in self.centros:
                        xs, ys = metros_a_pixeles(self, punto_guardado[0], punto_guardado[1])
                        x_pixel = xs
                        y_pixel = ys

                        if self.ruta is not None:
                            for punto_guardado1 in self.ruta:
                                xs, ys = metros_a_pixeles(self, punto_guardado1[0], punto_guardado1[1])
                                x_pixel_r = xs
                                y_pixel_r = ys
                                imagen_color[y_pixel_r, x_pixel_r] = [0, 150, 0]  # Verde

                        imagen_con_ruta[y_pixel, x_pixel] = [255, 0, 255]  # Magenta
                
                nombre_imagen = "imagen_con_ruta" + str(self.numero_de_imagen)
                nombre_imagen_con_extension = nombre_imagen + ".png"
                ruta_completa = os.path.join(carpeta, nombre_imagen_con_extension)
                cv2.imwrite(ruta_completa, imagen_con_ruta)  # Guarda la imagen con la ruta y los puntos aleatorios dibujados.
                ruta_completa = os.path.join(carpeta, "imagen_ruta.png")
                cv2.imwrite(ruta_completa, imagen_con_ruta)
                print(path)
                return ruta_optima
            path = None

            #self.meta_anterior = punto_destino
            punto_destino = (y_end,x_end)
            self.meta.append(punto_destino)
            img_color = cv2.cvtColor(imagen_reducida, cv2.COLOR_GRAY2BGR)

            punto_destino = generar_puntos_aleatorios_al_no_encotrar_ruta(self, imagen, img_color)
            self.plo = punto_destino
            self.plo_x, self.plo_y = pixeles_a_metros(self, self.plo[0], self.plo[1])
            
            imagen_color[punto_destino[1], punto_destino[0]] = [150, 255, 0]  # Marca el nuevo destino en la imagen
            y=punto_destino[1]
            x=punto_destino[0]
            punto_destino=[y,x]

            nombre_imagen = "new_meta" + str(self.num_new_img)
            self.num_new_img += 1
            nombre_imagen_con_extension = nombre_imagen + ".jpg"
            ruta_completa = os.path.join(carpeta, nombre_imagen_con_extension)
            

            cv2.imwrite(ruta_completa, imagen_color)

        
        except Exception as e:
            print(f"Error al encontrar la ruta: {e}")
            path = None
            punto_destino = (y_end,x_end)
            self.meta.append(punto_destino)
            img_color = cv2.cvtColor(imagen_reducida, cv2.COLOR_GRAY2BGR)

            punto_destino = generar_puntos_aleatorios_al_no_encotrar_ruta(self, imagen, img_color)
            self.plo = punto_destino
            self.plo_x, self.plo_y = pixeles_a_metros(self, self.plo[0], self.plo[1])
            self.plo_en_metros = pixeles_a_metros(self, self.plo[0], self.plo[1])
            
            imagen_color[punto_destino[1], punto_destino[0]] = [150, 255, 0]  # Marca el nuevo destino en la imagen
            y=punto_destino[1]
            x=punto_destino[0]
            punto_destino=[y,x]

            nombre_imagen = "new_meta_2_" + str(self.num_new_img)
            self.num_new_img += 1
            nombre_imagen_con_extension = nombre_imagen + ".jpg"
            ruta_completa = os.path.join(carpeta, nombre_imagen_con_extension)
            

            cv2.imwrite(ruta_completa, imagen_color)

           

def control_velocidad(self):
    k=0.5; 
    self.h=0.01    
    tetha=self.yaw

    self.xh = self.x+self.h*cos(tetha)
    self.yh = self.y+self.h*sin(tetha)

    ex = self.xh-self.punto_desado_xh;  ey = self.yh-self.punto_desado_yh
    Ux = -k*ex;  Uy =-k*ey
    self.Velocidad_Lineal= Ux*cos(tetha)+Uy*sin(tetha)
    self.Velocidad_Angular=-Ux*sin(tetha)/self.h+Uy*cos(tetha)/self.h
    
    #Evitar la saturacion en las velocidades 
    if (abs(self.Velocidad_Lineal)>V_max):
        self.Velocidad_Lineal = V_max*abs(self.Velocidad_Lineal)/self.Velocidad_Lineal
    if (abs(self.Velocidad_Angular)>W_max):
        self.Velocidad_Angular = W_max*abs(self.Velocidad_Angular)/self.Velocidad_Angular


class Nodo(object):
    def __init__(self):
        self.loop_rate = rospy.Rate(30)

        #Variables
        self.x = None
        self.velocidad=Twist()
        self.Velocidad_Lineal=0
        self.Velocidad_Angular=0
        self.punto_desado_xh=None
        self.plo_x=0; self.plo_y=0
        self.xh=0
        self.yh=0
        self.nodos=30
        self.ultimo=1
        self.crecio=False
        self.termina=False
        
        self.path=None
        self.puntos_guardados = []  # Vector para almacenar los puntos generados
        self.ruta = []
        self.path_metros=[]
        self.contador=1
        self.bandera_espacios=False
        self.limite=40
        self.lectura=0
        self.pixeles_anterior=0
        self.map_file = "map" 
        self.numero_de_imagen=0
        self.contador_nueva_meta=0
        self.centros = []  # Vector para almacenar los puntos generados
        self.plo_ant=[100,200]
        self.plo=[0,0]


      
        self.num_new_img=0
        self.fin=False
        self.meta=[]




        # Subscribirse
        rospy.Subscriber('/map', OccupancyGrid, self.map_callback)        
        odom_sub =rospy.Subscriber('/odom',Odometry, self.Callback)
        self.ph_pub =rospy.Publisher('/punto_h',Odometry, queue_size=10)
        self.odom_msg = Odometry()
        self.pub = rospy.Publisher('/cmd_vel',Twist,queue_size=10)
        self.publisher = rospy.Publisher("goal_point", MarkerArray, queue_size=3)
        


    def Callback(self,msg):
        #Posicion
        self.x=msg.pose.pose.position.x
        self.y=msg.pose.pose.position.y
        print("X={0} Y={1}".format(self.x,self.y))
        print("Xh={0} Yh={1}".format(self.xh,self.yh))
        print("Xhd={0} Yhd={1}".format(self.punto_desado_xh,self.punto_desado_yh))
        print("Plo={0} Plo={1}".format(self.plo_x,self.plo_y))
        #print("Plo_bizectriz={0} Plo_bizectriz={1}".format(self.plo_bizectriz[1],self.plo_bizectriz[0]))
        #Inicializa velocidad
        self.velocidad=Twist()
        #Orientacion
        quater = msg.pose.pose.orientation
        quater_list = [quater.x, quater.y, quater.z, quater.w]
        (roll, pitch, self.yaw) = euler_from_quaternion(quater_list)
        #Funcion de control
        control_velocidad(self)
        publish_markers(self)
    
    def map_callback(self, msg):
        # Establecer un flag para indicar que estamos procesando el mapa
        self.processing_map = True

        try:
            # Obtener la resolución, origen y dimensiones del mapa
            resolution = msg.info.resolution
            origen_x = msg.info.origin.position.x
            self.origen_y = msg.info.origin.position.y
            self.width = msg.info.width
            self.height = msg.info.height

            # Dimensiones del mapa en metros
            self.x_min = origen_x
            self.x_max  = origen_x + (self.width * resolution)
            self.y_min = self.origen_y
            self.y_max = self.origen_y + (self.height * resolution)

            # Crear un listener de tf
            listener = tf.TransformListener()

            # Esperar hasta que la transformación de /map a /base_link esté disponible
            listener.waitForTransform('/map', '/base_link', rospy.Time(0), rospy.Duration(3.0))
            
            # Obtener la transformación de /map a /base_link
            (trans, rot) = listener.lookupTransform('/map', '/base_link', rospy.Time(0))
            
            # La posición del robot en el marco de referencia /map
            self.robot_x = trans[0]
            self.robot_y = trans[1]
            
            
            if self.lectura == 0:
                self.punto_desado_xh = self.robot_x
                self.punto_desado_yh = self.robot_y

                self.plo_x = self.robot_x
                self.plo_y = self.robot_y

                self.inicio_x = self.robot_x
                self.inicio_y = self.robot_y

                self.plo_en_metros=[self.robot_x,self.robot_y]
                self.path_metros=[(self.robot_x,self.robot_y),(self.robot_x,self.robot_y)]
                

            # Esperar hasta que la transformación de /map a /odom esté disponible
            listener.waitForTransform('/map', '/odom', rospy.Time(0), rospy.Duration(1.0))
            
            # Obtener la transformación de /map a /odom
            (trans, rot) = listener.lookupTransform('/odom', '/map', rospy.Time(0))
            
            # Calcular el Local Origin en el marco global /odom
            self.origen_global_x = origen_x + trans[0]
            self.origen_global_y = self.origen_y + trans[1]

            self.resx = (abs(self.x_min) + abs(self.x_max )) / self.width
            self.resy = (abs(self.y_min) + abs(self.y_max)) / self.height


            self.pixel_origen_x=round(abs(self.origen_global_x/self.resx))#width-round(abs(origen_x/resx))
            self.pixel_origen_y=self.height-round(abs(self.origen_global_y/self.resy))#height-round(abs(origen_y/resy))
            
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logerr("Error de transformación: %s", str(e))
        
        finally:
            # Marcar que hemos terminado de procesar el mapa
            self.processing_map = False


    def start(self):
        random.seed(20)
        rospy.loginfo("Comenzando codigo")
        numero_de_muestras=2000
        #numero_de_muestras=200
        distancia_k_vecinos=10

        
        # Obtén la carpeta personal del usuario
        carpeta_raiz = os.path.expanduser("~")

        # Define la ruta a la carpeta "pruebas"
        carpeta_pruebas = os.path.join(carpeta_raiz, 'Pruebas_explore')
        carpeta = os.path.join(carpeta_pruebas, 'Prueba_plano_2')

        
        if not os.path.exists(carpeta):
            os.makedirs(carpeta)
        
        print(self.contador > self.limite)
        while (not rospy.is_shutdown() and self.contador < self.limite):# or self.termina==True:
            if self.x is not None and self.punto_desado_xh is not None:
                
                #Asigna la velocidad y el angulo del robot
                self.velocidad.linear.x = self.Velocidad_Lineal 
                self.velocidad.angular.z = self.Velocidad_Angular
                self.pub.publish(self.velocidad)
                #Publicación de ph en RVIZ
                self.odom_msg.pose.pose.position.x = self.xh
                self.odom_msg.pose.pose.position.y = self.yh
                self.ph_pub.publish(self.odom_msg)


                distancia_plo = sqrt((self.x - self.plo_x)**2 + (self.y - self.plo_y)**2)
                
                if distancia_plo < 0.15:
                    
                    
                 #Obtener imagen del mapa
                    print("plo")
                    self.meta=[]
                    self.Velocidad_Lineal=0
                    self.Velocidad_Angular=0
                    self.velocidad.linear.x = self.Velocidad_Lineal 
                    self.velocidad.angular.z = self.Velocidad_Angular
                    #numero_de_muestras=1500
                    #distancia_k_vecinos=50
                    #numero_de_muestras=numero_de_muestras+500
                    numero_de_muestras=numero_de_muestras+50
                    #distancia_k_vecinos=distancia_k_vecinos+1
                    ruta_completa = os.path.join(carpeta, self.map_file)
                    self.numero_de_imagen = self.numero_de_imagen + 1
                
                    cmd = ["rosrun", "map_server", "map_saver", "-f", ruta_completa]
                    proc = subprocess.Popen(cmd)
                    proc.wait()
                    
                    # Carga la imagen del mapa
                    imagen_original = cv2.imread(ruta_completa+ '.pgm', cv2.IMREAD_GRAYSCALE)
                    umbral = 240  
                    _, imagen_binaria = cv2.threshold(imagen_original, umbral, 255, cv2.THRESH_BINARY)
                    
                    # Guardar la imagen_original binarizada
                    nombre_imagen = "imagen_binarizada" + str(self.numero_de_imagen)
                    nombre_imagen_con_extension = nombre_imagen + ".png"
                    ruta_completa = os.path.join(carpeta, nombre_imagen_con_extension)

                    cv2.imwrite(ruta_completa, imagen_binaria)                       

                    self.lectura=1
                    self.nodos=self.nodos+5

                    kernel = np.ones((9,9),np.uint8)
                    img_bin = cv2.erode(imagen_binaria,kernel,iterations = 1)

                    kernel = np.ones((12,12),np.uint8)
                    img = cv2.erode(imagen_binaria,kernel,iterations = 1)

                    kernel = np.ones((13,13),np.uint8)
                    imagen_reducida = cv2.erode(imagen_binaria,kernel,iterations = 1)

                    imagen_color = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
                    imagen_color2 = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
                    imagen_origen = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
                    sp,j=metros_a_pixeles(self,self.robot_x,self.robot_y)

                    imagen_origen[self.pixel_origen_y,self.pixel_origen_x] = [0, 0, 255]  # Morado

                    ruta_completa = os.path.join(carpeta, "inicio.jpg")
                    cv2.imwrite(ruta_completa, imagen_origen)
                                        
                    #xd,yd=metros_a_pixeles(self,self.meta_x,self.meta_y)
                    #print(xd,yd)

                    
                    
                    pixeles_blancos = np.sum(imagen_binaria == 255)
                    print(pixeles_blancos)
                    porcentaje_diferencia = ((pixeles_blancos - self.pixeles_anterior) / self.pixeles_anterior) * 100
                    print(porcentaje_diferencia)

                    if porcentaje_diferencia < 10:
                        self.contador += 1
                        print("self.contador")
                        print(self.contador)
                        print(self.ultimo)
                        print("self.ultimo")

                    if self.contador==2:
                        p1, p2 = get_white_points(self,imagen_binaria,False)
                        p3=(self.pixel_origen_y,self.pixel_origen_x)
                        p4=generar_puntos_aleatorios(self, imagen_reducida, imagen_color)
                        p3=(p4[1],p4[0])

                        image_with_bisector,bisector_point = calculate_bisector(imagen_binaria, p1, p2, p3)
                        image_with_bisector,bisector_point = calculate_bisector(imagen_binaria, p1, p2, p3)
                        plo=(int(bisector_point[0]),int(bisector_point[1]))
                        print(plo)

                        px_bis_y, px_bis_x =pixeles_a_metros(self, plo[0], plo[1])
                        self.plo_bizectriz=[px_bis_x,px_bis_y]
                        self.plo=plo
                        #self.plo=[px_bis_y, px_bis_x]
                        nombre_imagen = "imagen_bisectriz" + str(self.contador) +"_"+ str(self.numero_de_imagen)
                        nombre_imagen_con_extension = nombre_imagen + ".png"
                        ruta_completa = os.path.join(carpeta, nombre_imagen_con_extension)
                        cv2.imwrite(ruta_completa, image_with_bisector)

                    elif self.contador>=3:
                        p1, p2 = get_white_points(self,imagen_binaria,True)
                        self.ultimo=self.ultimo+1

                        p3=(self.pixel_origen_y,self.pixel_origen_x)
                        p4=generar_puntos_aleatorios(self, imagen_reducida, imagen_color)
                        p3=(p4[1],p4[0])

                        image_with_bisector,bisector_point = calculate_bisector(imagen_binaria, p1, p2, p3)
                        image_with_bisector,bisector_point = calculate_bisector(imagen_binaria, p1, p2, p3)
                        plo=(int(bisector_point[0]),int(bisector_point[1]))
                        print(plo)
                        px_bis_y, px_bis_x =pixeles_a_metros(self, plo[0], plo[1])
                        self.plo_bizectriz=[px_bis_x,px_bis_y]
                        self.plo=plo
                        nombre_imagen = "imagen_bisectriz_ultimo" + str(self.ultimo)+"_"+str(self.numero_de_imagen)
                        nombre_imagen_con_extension = nombre_imagen + ".png"
                        ruta_completa = os.path.join(carpeta, nombre_imagen_con_extension)
                        cv2.imwrite(ruta_completa, image_with_bisector)
                        if porcentaje_diferencia > 3:
                            self.crecio = True
                            print("self.crecio")
                            print(self.crecio)
                            self.contador=0
                            self.ultimo=1
                            print(self.ultimo)
                            print("self.ultimo")
                        else:
                            self.crecio = False
                    else:  
                        self.plo = generar_puntos_aleatorios_radio(self, imagen_reducida, imagen_color) 
                    print(self.plo_ant)
                    print(self.plo)

                    if self.plo_ant==self.plo:
                        self.termina=True
                        self.contador=self.limite

                    print(self.ultimo)
                    print("self.ultimo")
                    if self.ultimo==9 and self.crecio==False:
                        self.termina=True
                        self.contador=self.limite
                    

                        

                    self.pixeles_anterior = pixeles_blancos
                    xs,ys=metros_a_pixeles(self, self.punto_desado_xh, self.punto_desado_yh)
                    start=(ys,xs)
                    print("No blanco")
                    
                    


                    imagen_origen[ys,xs] = [0, 100, 255]  # Morado

                    
                    
                    self.plo_x, self.plo_y = pixeles_a_metros(self, self.plo[0], self.plo[1])
                    
                    #self.plo=(int(bisector_point[0]),int(bisector_point[1]))
                    #print(plo)
                    y_end, x_end = encontrar_pixel_blanco_mas_cercano(imagen_reducida, self.plo[1], self.plo[0])
                    self.plo_x, self.plo_y =pixeles_a_metros(self, x_end, y_end)

                    goal=(self.plo[1], self.plo[0])


                    imagen_origen[start] = [0, 0, 255]  # Morado
                    imagen_origen[goal] = [255,0,0]

                    
                    
                    self.path=prm(self, img,imagen_reducida, img_bin, imagen_color, numero_de_muestras, distancia_k_vecinos, start, goal,carpeta)
                    nombre_imagen = "imagen_origen" + str(self.numero_de_imagen)
                    nombre_imagen_con_extension = nombre_imagen + ".png"
                    ruta_completa = os.path.join(carpeta, nombre_imagen_con_extension)
                    self.meta=[]
                    cv2.imwrite(ruta_completa, imagen_origen)

                    self.plo_en_metros = pixeles_a_metros(self, self.plo[1], self.plo[0])
                    print("METROOOOOOOOOOOOOOOS")
                    self.centros.append(self.plo_en_metros)
                    self.ruta.extend(self.path_metros)
                    print(self.plo_en_metros)
                    bandera=0
                    self.path_metros= [pixeles_a_metros(self, px, py) for py, px in self.path]
                    
                    print(self.path_metros)
                    #except:
                    #    print("No hubo ruta")
                        
                    
                    nombre_imagen = "imagen_" + str(self.numero_de_imagen)
                    nombre_imagen_con_extension = nombre_imagen + ".png"
                    ruta_completa = os.path.join(carpeta, nombre_imagen_con_extension)
                        
                    cv2.imwrite(ruta_completa, imagen_color)
                    self.plo_ant=self.plo

                    
                    
                distancia_control = sqrt((self.x - self.punto_desado_xh)**2 + (self.y - self.punto_desado_yh)**2)

                # Distancia de tolerancia al punto deseado 
                if distancia_control < 0.08:
                    sp,j=metros_a_pixeles(self,-1,2)
                    rp,t=pixeles_a_metros(self,sp,j)

                    if bandera == (len(self.path_metros)-1):
                        self.Velocidad_Lineal=0
                        self.Velocidad_Angular=0
                        self.velocidad.linear.x = self.Velocidad_Lineal 
                        self.velocidad.angular.z = self.Velocidad_Angular
                        self.pub.publish(self.velocidad)
                        print("pausaaaaaaaaaaa")
                        #time.sleep(5)

                    if bandera < len(self.path_metros):
                        self.punto_desado_xh= self.path_metros[bandera][0]
                        self.punto_desado_yh = self.path_metros[bandera][1]
                        
                        bandera=bandera+1
                        print("posicion xd")
                        print(self.punto_desado_xh, self.punto_desado_yh)
                    
                    
                control_velocidad(self)
                
            self.loop_rate.sleep()


if __name__ == '__main__':
    rospy.init_node("PRM_SLAM", anonymous=True)


    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)

    try:
        my_node = Nodo()
        my_node.start()
    except rospy.ROSInterruptException:
        pass
    
    