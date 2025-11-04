#!/usr/bin/env python3

# Importaciones clave de ROS2 y dependencias
import rclpy
from rclpy.node import Node
from djitellopy import Tello
import cv_bridge
import cv2

# Importamos los tipos de mensajes
from sensor_msgs.msg import Image       # Para el video
from std_msgs.msg import Int32          # Para la batería, velocidad, altura
from std_msgs.msg import String         # <--- ¡NUEVO! Para recibir comandos

class DroneConnectorNode(Node):
    """
    Este nodo actúa como el "driver" principal.
    Se conecta al Tello, enciende el stream de video, publica telemetría y
    ejecuta los comandos recibidos por tópicos de ROS2.
    """
    def __init__(self):
        super().__init__('drone_connector')
        self.get_logger().info('Iniciando el nodo Conector del Tello...')

        # --- 1. Conexión con el Tello ---
        self.tello = Tello()
        try:
            self.tello.connect()
            self.get_logger().info('¡Conexión con el Tello exitosa!')
        except Exception as e:
            self.get_logger().error(f'No se pudo conectar con el Tello: {e}')
            # Si no podemos conectarnos, cerramos el nodo.
            rclpy.shutdown()
            return

        # --- 2. Iniciar el stream de video ---
        self.tello.streamon()
        self.get_logger().info('Stream de video del Tello iniciado.')
        
        # Obtenemos el "lector" de fotogramas del Tello
        self.frame_reader = self.tello.get_frame_read()

        # --- 3. Inicializar el CV-Bridge ---
        self.bridge = cv_bridge.CvBridge()

        # --- 4. Declarar Publicadores (Publishers) ---
        self.image_pub = self.create_publisher(Image, '/tello/image_raw', 10)
        self.battery_pub = self.create_publisher(Int32, '/tello/battery', 10)
        self.speed_x_pub = self.create_publisher(Int32, '/tello/speed_x', 10)
        self.height_pub = self.create_publisher(Int32, '/tello/height', 10)

        # --- 5. Declarar Suscriptor (Subscriber) de Comandos (¡CLAVE!) ---
        self.command_sub = self.create_subscription(
            String,
            '/tello/command',
            self.command_callback,
            10
        )
        self.get_logger().info('Listo para recibir comandos en /tello/command.')

        # --- 6. Crear Timers (Temporizadores) ---
        video_fps = 60.0
        self.video_timer = self.create_timer(1.0 / video_fps, self.video_callback)
        self.telemetry_timer = self.create_timer(2.0, self.telemetry_callback)

        self.get_logger().info('Nodo Conector listo y publicando datos.')


    # --------------------------------------------------
    # --- CALLBACKS DE SUSCRIPCIÓN ---
    # --------------------------------------------------

    def command_callback(self, msg):
        """
        [NUEVO] Ejecuta los comandos de vuelo recibidos del MissionPlanner y Failsafe.
        """
        command = msg.data.strip().lower()
        self.get_logger().info(f'Ejecutando comando Tello: "{command}"')
        
        try:
            # Comandos simples
            if command == "takeoff":
                self.tello.takeoff()
            elif command == "land":
                self.tello.land()
            elif command == "emergency":
                self.tello.emergency()
                
            # Comandos de movimiento con valor (ej. forward 50, up 30)
            elif len(command.split()) == 2:
                cmd, val = command.split()
                val = int(val)
                
                if cmd == "forward":
                    self.tello.move_forward(val)
                elif cmd == "back":
                    self.tello.move_back(val)
                elif cmd == "up":
                    self.tello.move_up(val)
                elif cmd == "down":
                    self.tello.move_down(val)
                elif cmd == "left":
                    self.tello.move_left(val)
                elif cmd == "right":
                    self.tello.move_right(val)
                elif cmd == "cw":
                    self.tello.rotate_clockwise(val)
                elif cmd == "ccw":
                    self.tello.rotate_counter_clockwise(val)
                else:
                    self.get_logger().warn(f'Comando de movimiento desconocido: "{command}"')
            
            # Puedes añadir manejo para comandos RC (ej. "rc 0 0 0 0") si es necesario
            
            else:
                self.get_logger().warn(f'Comando desconocido o mal formateado: "{command}"')
                
        except Exception as e:
            self.get_logger().error(f'Error al ejecutar el comando Tello "{command}": {e}')


    # --------------------------------------------------
    # --- CALLBACKS DE TIMER (PUBLICACIÓN) ---
    # --------------------------------------------------

    def video_callback(self):
        # ... (cuerpo de la función sin cambios) ...
        try:
            frame = self.frame_reader.frame
            if frame is not None:
                img_msg = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
                img_msg.header.stamp = self.get_clock().now().to_msg()
                self.image_pub.publish(img_msg)
        except Exception as e:
            self.get_logger().warn(f'Error al procesar el fotograma de video: {e}')

    def telemetry_callback(self):
        # ... (cuerpo de la función sin cambios) ...
        try:
            # Obtenemos la batería
            battery = self.tello.get_battery()
            
            # Publicamos la batería
            battery_msg = Int32()
            battery_msg.data = battery
            self.battery_pub.publish(battery_msg)
            
            self.get_logger().info(f'Estado de conexión: OK (Batería: {battery}%)')
            
            # Velocidad (X)
            speed_x = self.tello.get_speed_x() # cm/s
            speed_msg = Int32()
            speed_msg.data = speed_x
            self.speed_x_pub.publish(speed_msg)

            # Altura
            height = self.tello.get_height() # cm
            height_msg = Int32()
            height_msg.data = height
            self.height_pub.publish(height_msg)

        except Exception as e:
            self.get_logger().warn(f'Error al obtener telemetría: {e}')


def main(args=None):
    rclpy.init(args=args)
    
    node = DroneConnectorNode()
    
    # 'spin' mantiene el nodo vivo para que los timers sigan funcionando
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Señal de cierre recibida.')
    finally:
        # Esto asegura que el Tello aterrice y apague el stream si el nodo se detiene
        # (ya sea por KeyboardInterrupt o por rclpy.shutdown() en el __init__)
        node.get_logger().info('Cerrando conexión con Tello: Land y Stream Off.')
        # Solo ejecutar si la conexión fue exitosa
        if hasattr(node, 'tello') and node.tello.is_flying:
             node.tello.land()
        if hasattr(node, 'tello'):
            node.tello.streamoff()
            
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
