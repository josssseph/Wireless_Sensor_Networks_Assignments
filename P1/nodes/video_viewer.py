# Nodo para la visualización de video desde el Drone Tello
#!/usr/bin/env python3

# Importaciones clave
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv_bridge
import cv2
import os

# --- INICIO DE LA SOLUCIÓN GRÁFICA DE DOCKER (Requerido por el PDF) ---
# Estas líneas le dicen a OpenCV que no use aceleración por hardware (GPU)
# y que utilice la plataforma "xcb" para la renderización, lo cual
# es compatible con el reenvío X11 de Docker.

# Asegura que OpenCV use XCB para la renderización de la GUI, no OpenGL.
os.environ["QT_QPA_PLATFORM"] = "xcb"
# Renderiza por software (CPU), deshabilitando la aceleración de hardware
# que entra en conflicto con Docker.
os.environ["LIBGL_ALWAYS_SOFTWARE"] = "1"
# --- FIN DE LA SOLUCIÓN GRÁFICA ---


class VideoViewerNode(Node):
    """
    Nodo 2: Visualizador de Video.
    - Se suscribe a /tello/image_raw.
    - Convierte el mensaje ROS a una imagen OpenCV.
    - Muestra el video en una ventana de cv2.imshow().
    """
    def __init__(self):
        super().__init__('video_viewer')
        self.get_logger().info('Iniciando el nodo Visualizador de Video.')
        self.get_logger().info('Esperando fotogramas en el tópico /tello/image_raw...')

        # Inicializa el CV-Bridge
        self.bridge = cv_bridge.CvBridge()

        # Crea la suscripción al tópico de video del Nodo 1
        self.subscription = self.create_subscription(
            Image,
            '/tello/image_raw',      # <-- Tópico publicado por el Nodo 1
            self.image_callback,    # <-- Función a ejecutar con cada fotograma
            10
        )

    def image_callback(self, msg):
        """
        Esta función se llama cada vez que se recibe un mensaje
        en el tópico /tello/image_raw.
        """
        try:
            # --- CORRECCIÓN DE COLOR AUTOMÁTICA ---
            # Pedimos 'bgr8' porque es lo que cv2.imshow() espera.
            # cv_bridge verá que el 'msg' está en 'rgb8' (del Nodo 1)
            # y automáticamente hará la conversión de color RGB -> BGR.
            frame = self.bridge.imgmsg_to_cv2(msg, 'rgb8')
            
            # Muestra el fotograma en una ventana llamada 'Tello Video'
            cv2.imshow('Tello Video', frame)
            
            # Esta línea es ESENCIAL para que la ventana se actualice.
            cv2.waitKey(1)

        except cv_bridge.CvBridgeError as e:
            self.get_logger().error(f'Error de CV-Bridge: {e}')
        except Exception as e:
            self.get_logger().error(f'Error al procesar el fotograma: {e}')

    def destroy_node(self):
        """Limpia los recursos al cerrar el nodo."""
        self.get_logger().info('Cerrando el nodo y la ventana de video.')
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    video_node = VideoViewerNode()
    
    try:
        # 'spin' mantiene el nodo vivo para que el suscriptor siga escuchando
        rclpy.spin(video_node)
    except KeyboardInterrupt:
        pass # El usuario presionó Ctrl+C
    finally:
        # Nos aseguramos de cerrar la ventana de OpenCV
        video_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
