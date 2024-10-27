import subprocess
import cv2
import numpy as np
# Comando a ejecutar
command = "rosrun map_server map_saver -f map"

try:
    while True:
        subprocess.run(command, shell=True, check=True)
        imagen = cv2.imread('map.pgm', cv2.IMREAD_GRAYSCALE)

        # Aplicar umbral para binarizar la imagen
        umbral = 220  
        _, imagen_binaria = cv2.threshold(imagen, umbral, 255, cv2.THRESH_BINARY)
        # Redimensionar la imagen a un tamaño específico (por ejemplo, 800x600)
        nuevo_ancho = 800
        nueva_altura = 600
        imagen_redimensionada = cv2.resize(imagen_binaria, (nuevo_ancho, nueva_altura))
        # Guardar la imagen binarizada
        cv2.imwrite('map.png', imagen_binaria)

        print("Comando ejecutado con éxito")
except KeyboardInterrupt:
    print("Interrupción manual. Finalizando la ejecución del programa.")
