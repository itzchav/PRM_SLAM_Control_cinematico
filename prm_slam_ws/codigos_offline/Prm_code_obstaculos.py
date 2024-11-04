import cv2
import networkx as nx
import numpy as np
import random
import matplotlib.pyplot as plt
import csv

from math import sin, cos, pi, sqrt

from scipy.spatial import KDTree

def pixeles_a_metros(pixel_x, pixel_y):
    # Calcula las coordenadas en metros a partir de las coordenadas en pixeles
    resx=(707-144)/28.6693
    resy=(702-133)/28.4363
    print(resy,resx)

    centro_y = 430   
    centro_x = 692
    
    metros_y = ((centro_y-pixel_y) / resy)
    metros_x = ((centro_x-pixel_x) / resx)
    
    print("Coordenadas en metros del punto:", metros_x, metros_y)
    
    return metros_x, metros_y

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

def hay_obstaculo_entre_puntos(punto1, punto2, imagen_binaria):
    # Verificar si hay obstáculos entre dos puntos utilizando la línea entre ellos
    linea = np.linspace(punto1, punto2, num=10).astype(int)
    for punto in linea:
        if imagen_binaria[punto[0], punto[1]] == 0:
            return True
    return False

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

def prm( imagen, numero_de_muestras, distancia_max):
    path = None

    #Prueba 1
    punto_inicio = (692, 430)
    punto_destino = (580,547)
    #Prueba 2
    punto_inicio = (527, 235)
    punto_destino = (257,281)
    #Prueba 3
    punto_inicio = (692, 430)
    punto_destino = (167, 430)
    #Prueba 4
    punto_inicio = (514,643)
    punto_destino = (124, 370)

    points = []  # Lista para almacenar los puntos aleatorios
    img_color = cv2.cvtColor(imagen, cv2.COLOR_GRAY2BGR)
    
    # Bucle para intentar encontrar una ruta
    while path is None:
        print("generating while")
         # Verificar y ajustar el punto de inicio
       
        grafo = nx.Graph()
        muestras_validas = generar_muestras_validas(imagen, numero_de_muestras)
        img_color[punto_inicio] = [255, 255, 0]  # Amarillo
        img_color[punto_destino] = [0, 255, 255]  # Cian

        cv2.imshow('imagen',img_color)
        cv2.waitKey(0)
 
        # Close all windows
        cv2.destroyAllWindows()

        muestras = np.vstack([muestras_validas, punto_inicio, punto_destino])  # Agregar puntos de inicio y destino
        kdtree = KDTree(muestras)
        for i, punto in enumerate(muestras):
            # Encontrar vecinos dentro de la distancia máxima
            distancias, vecinos_indices = kdtree.query(punto, k=25, distance_upper_bound=distancia_max)
            for vecino_indice, distancia in zip(vecinos_indices, distancias):
                if distancia < distancia_max and not hay_obstaculo_entre_puntos(punto, muestras[vecino_indice], imagen):
                    grafo.add_edge(tuple(punto), tuple(muestras[vecino_indice]), weight=distancia)

        # Dibuja la ruta en la imagen original.
        
        img_color = cv2.cvtColor(imagen, cv2.COLOR_GRAY2BGR)
        imagen_con_ruta = img_color.copy()  # Copia la imagen original.
        
        im=dibujar_grafo(img_color, grafo, muestras)
        cv2.imwrite("grafo4.jpg", im)  # Guarda la imagen con la ruta y los puntos aleatorios dibujados.
           
        if nx.has_path(grafo, tuple(punto_inicio), tuple(punto_destino)):
            ruta_optima = nx.shortest_path(grafo, source=tuple(punto_inicio), target=tuple(punto_destino), weight='weight')
            print(len(ruta_optima))

            punto_inicio = pixeles_a_metros(punto_inicio[0], punto_inicio[1])
            punto_destino = pixeles_a_metros(punto_destino[0], punto_destino[1])
            distancia_total = sqrt((punto_inicio[0] - punto_destino[0])**2 + (punto_inicio[1] - punto_destino[1])**2)

            path = ruta_optima
            for i in range(len(path) - 1):
                point1 = (path[i][1], path[i][0])  # Intercambiar coordenadas xy a yx
                point2 = (path[i + 1][1], path[i + 1][0])  # Intercambiar coordenadas xy a yx
                cv2.line(imagen_con_ruta, point1, point2, (0, 0, 255), 1)  # Dibuja la línea roja entre nodos de la ruta.
                points.append(point2)  # Agregar puntos aleatorios a la lista
             

            # Dibuja los puntos aleatorios en la imagen
            for point in points:
                cv2.circle(imagen_con_ruta, point, 2, (0, 255, 0), -1)  # Dibuja un círculo verde en cada punto aleatorio
            punto_destino=(punto_destino[1],punto_destino[0])
            points.append(punto_destino)

            cv2.imwrite("prueba7_obs_1200_500.jpg", imagen_con_ruta)  # Guarda la imagen con la ruta y los puntos aleatorios dibujados.

            # Guardar los puntos aleatorios en un archivo CSV
            with open("puntos7_obs_1200_500.csv", "w", newline="") as csvfile:
                csvwriter = csv.writer(csvfile)
                csvwriter.writerow(["X", "Y"])
                for point in points:
                    x,y=pixeles_a_metros(point[0],point[1])
                    print(x,y)
                    csvwriter.writerow((x,y))
                    #csvwriter.writerow(point)

            with open("prueba7_obs_1200_500.csv", "w", newline="") as csvfile2:
                csvwriter = csv.writer(csvfile2)
                csvwriter.writerow(["X", "Y"])
                for i in range(len(path)):
                    x,y=pixeles_a_metros(path[i][0],path[i][1])
                    print(x,y)
                    csvwriter.writerow((x,y))
                    #csvwriter.writerow(point)

            return path
        else:
            return None
            

# Carga la imagen en escala de grises.
imagen = cv2.imread("map_obstaculos.png", cv2.IMREAD_GRAYSCALE)
imagen_invertida = cv2.bitwise_not(imagen)

kernel = np.ones((12,12),np.uint8)
dilatacion = cv2.dilate(imagen_invertida,kernel,iterations = 1)
kernel = np.ones((1,1),np.uint8)
cierre = cv2.morphologyEx(dilatacion, cv2.MORPH_CLOSE, kernel)


cv2.imshow('imagen',cierre)
# Establece un umbral para la binarización.
umbral = 128  # Puedes ajustar este valor según tus necesidades.
imagen_inv = cv2.bitwise_not(cierre)
imagen[400,107]=255
cv2.imshow('imagen',imagen_inv)

# Aplica la binarización.
image = np.where(imagen_inv >= umbral, 255, 0)

num_samples = 1200
k_neighbors = 500
path = prm(imagen_inv, num_samples, k_neighbors)
print(path)
