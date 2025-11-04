# Nodo para monitorizar las stats del Drone Tello

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import os # Lo usaremos para limpiar la pantalla

class TelemetryMonitorNode(Node):
    """
    Nodo 3: Monitor de Telemetría.
    - Se suscribe a /tello/battery, /tello/speed_x, /tello/height
    - Muestra los datos en la consola de forma clara y legible.
    """
    def __init__(self):
        super().__init__('telemetry_monitor')
        self.get_logger().info('Iniciando el monitor de telemetría...')

        # Almacenamiento para los últimos valores recibidos
        self.latest_battery = -1
        self.latest_speed_x = 0
        self.latest_height = 0

        # Suscriptores
        self.battery_sub = self.create_subscription(
            Int32, '/tello/battery', self.battery_callback, 10)
        
        self.speed_sub = self.create_subscription(
            Int32, '/tello/speed_x', self.speed_callback, 10)
        
        self.height_sub = self.create_subscription(
            Int32, '/tello/height', self.height_callback, 10)
        
        # Un temporizador para imprimir el 'dashboard' 2 veces por segundo (2 Hz)
        # Usamos un timer para que la pantalla se actualice a un ritmo constante
        self.print_timer = self.create_timer(0.5, self.print_dashboard)

    # Callbacks para actualizar nuestros valores
    def battery_callback(self, msg):
        self.latest_battery = msg.data

    def speed_callback(self, msg):
        self.latest_speed_x = msg.data

    def height_callback(self, msg):
        self.latest_height = msg.data

    def print_dashboard(self):
        """
        Limpia la consola y muestra los últimos datos de telemetría.
        Cumple el requisito de "clara y legible, actualizándose constantemente".
        """
        # Limpiar la consola (funciona en Linux/macOS)
        os.system('clear')
        
        self.get_logger().info('--- MONITOR DE TELEMETRÍA TELLO ---')
        self.get_logger().info(f'     Batería : {self.latest_battery} %')
        self.get_logger().info(f'     Altura  : {self.latest_height} cm')
        self.get_logger().info(f'  Velocidad (X) : {self.latest_speed_x} cm/s')
        self.get_logger().info('-------------------------------------')
        self.get_logger().info('(Presiona Ctrl+C para salir)')


def main(args=None):
    rclpy.init(args=args)
    
    node = TelemetryMonitorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        # Imprimir una línea vacía al final para no arruinar la terminal
        print("\nMonitor de telemetría detenido.")
        rclpy.shutdown()

if __name__ == '__main__':
    main()
