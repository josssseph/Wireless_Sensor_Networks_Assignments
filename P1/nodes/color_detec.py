#!/usr/bin/env python3

# Hacks de GUI para Docker, ¡requeridos por el PDF!
import os
os.environ["QT_QPA_PLATFORM"] = "xcb"
os.environ["LIBGL_ALWAYS_SOFTWARE"] = "1"

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image     # Para suscribirse
from std_msgs.msg import Int32        # Para publicar el conteo
from cv_bridge import CvBridge
import cv2
import numpy as np # Necesario para el manejo de rangos de color

class ObjectDetectorNode(Node):
    """
    Nodo 6: Detección de Objetos.
    - Se suscribe a /tello/image_raw (publicado por Nodo 1).
    - Detecta objetos rojos y negros usando HSV.
    - Dibuja rectángulos en los objetos detectados.
    - Muestra el video procesado en una ventana de OpenCV.
    - Publica el conteo de objetos en /tello/objects_count.
    """
    
    def __init__(self):
        super().__init__('object_detector_node')
        self.get_logger().info('Iniciando el nodo de detección de objetos.')
        self.bridge = CvBridge()
        
        # --- 1. Definir Rangos de Color HSV ---
        # (Este es el núcleo de la lógica de detección)
        
        # Rango para NEGRO (Valores bajos de V)
        self.lower_black = np.array([0, 0, 0])
        self.upper_black = np.array([180, 255, 30])
        
        # Rango para ROJO (envuelve el inicio/fin del espectro H)
        # Rango 1 (0-10)
        self.lower_red1 = np.array([0, 70, 50])
        self.upper_red1 = np.array([10, 255, 255])
        # Rango 2 (170-180)
        self.lower_red2 = np.array([170, 70, 50])
        self.upper_red2 = np.array([180, 255, 255])

        # --- 2. Suscriptor de Imagen ---
        self.image_sub = self.create_subscription(
            Image,
            '/tello/image_raw',
            self.image_callback,
            10
        )
        
        # --- 3. Publicador de Conteo ---
        self.objects_pub = self.create_publisher(Int32, '/tello/objects_count', 10)
        
        # Área mínima para filtrar ruido (pixeles)
        self.min_area = 500

    def image_callback(self, msg):
        """Callback principal: procesa cada frame de video."""
        try:
            # Convertimos el mensaje ROS a imagen OpenCV (BGR)
            # El Nodo 1 publica 'rgb8', cv_bridge lo convierte a 'bgr8'
            # que es lo que OpenCV espera por defecto.
            frame = self.bridge.imgmsg_to_cv2(msg, 'rgb8')
        except Exception as e:
            self.get_logger().error(f'Error al convertir imagen: {e}')
            return
            
        # Convertir de BGR a HSV (Espacio de Color H-S-V)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # --- Crear Máscaras ---
        mask_black = cv2.inRange(hsv, self.lower_black, self.upper_black)
        
        mask_red1 = cv2.inRange(hsv, self.lower_red1, self.upper_red1)
        mask_red2 = cv2.inRange(hsv, self.lower_red2, self.upper_red2)
        mask_red = cv2.bitwise_or(mask_red1, mask_red2)
        
        # Combinar ambas máscaras (rojo O negro)
        total_mask = cv2.bitwise_or(mask_red, mask_black)
        
        # --- Encontrar Contornos ---
        # (Encontrar los "perfiles" de los objetos aislados)
        contours, _ = cv2.findContours(total_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        object_count = 0
        
        # --- Filtrar y Dibujar ---
        for cnt in contours:
            area = cv2.contourArea(cnt)
            
            # Filtrar por área para eliminar ruido
            if area > self.min_area:
                object_count += 1
                
                # Dibujar el rectángulo
                x, y, w, h = cv2.boundingRect(cnt)
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
        
        # --- Mostrar Contador en Pantalla (Requisito del PDF) ---
        cv2.putText(
            frame, 
            f'Objetos detectados: {object_count}', 
            (10, 30), # Posición (x, y)
            cv2.FONT_HERSHEY_SIMPLEX, 
            1, (0, 255, 0), 2
        )
        
        # Mostrar el frame procesado en una ventana
        cv2.imshow("Deteccion de Objetos (Nodo 6)", frame)
        cv2.waitKey(1) # Requerido para que la ventana se actualice
        
        # --- Publicar Conteo (Requisito del PDF) ---
        count_msg = Int32()
        count_msg.data = object_count
        self.objects_pub.publish(count_msg)

def main(args=None):
    rclpy.init(args=args)
    
    node = ObjectDetectorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('Cerrando nodo de detección.')
        cv2.destroyAllWindows() # Cierra la ventana de OpenCV
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
