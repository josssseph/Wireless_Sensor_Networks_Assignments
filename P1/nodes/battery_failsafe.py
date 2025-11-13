# Nodo para secuencia de aterrizaje seguro en caso de baja batería
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32    # Para suscribirse a la batería
from std_msgs.msg import String   # Para publicar comandos
from std_msgs.msg import Bool     # Para publicar el estado de seguridad

class BatteryFailsafeNode(Node):
    """
    Nodo 4: Lógica de Batería Segura.
    - Se suscribe a /tello/battery.
    - Publica el estado de seguridad en /tello/safety_status (Bool).
    - Si la batería < 30%, publica "land" en /tello/command.
    """
    
    # Definimos el umbral de batería como un parámetro de clase
    BATTERY_THRESHOLD = 70 # Umbral del 30% como pide el PDF

    def __init__(self):
        super().__init__('battery_failsafe')
        self.get_logger().info(f'Iniciando el nodo de seguridad. Umbral: {self.BATTERY_THRESHOLD}%')
        
        # Para evitar enviar "land" 100 veces por segundo.
        # Solo lo enviamos una vez cuando se cruza el umbral.
        self.failsafe_triggered = False

        # Suscriptor al tópico de batería (publicado por Nodo 1)
        self.battery_sub = self.create_subscription(
            Int32,
            '/tello/battery',
            self.battery_callback,
            10
        )

        # Publicador del comando (escuchado por Nodo 1)
        self.command_pub = self.create_publisher(
            String,
            '/tello/command',
            10
        )
        
        # Publicador del estado de seguridad (para que el Nodo 5 lo lea)
        self.safety_pub = self.create_publisher(
            Bool,
            '/tello/safety_status',
            10 # '10' es el "queue size"
        )

    def battery_callback(self, msg):
        battery_level = msg.data
        is_safe = (battery_level >= self.BATTERY_THRESHOLD)
        
        # 1. Publicar siempre el estado de seguridad (para el Nodo 5)
        safety_msg = Bool()
        safety_msg.data = is_safe
        self.safety_pub.publish(safety_msg)

        # 2. Lógica de Failsafe (Aterrizaje)
        if is_safe:
            # Si la batería está bien (ej. se recargó), reseteamos el failsafe
            if self.failsafe_triggered:
                self.get_logger().info(f'Batería OK ({battery_level}%). Failsafe reseteado.')
            self.failsafe_triggered = False
        else:
            # La batería está baja
            if not self.failsafe_triggered:
                # Si es la PRIMERA vez que detectamos batería baja, activamos el failsafe
                self.get_logger().warn(f'¡FALLO DE BATERÍA! Nivel: {battery_level}%. ¡Enviando comando de aterrizaje!')
                
                # Publicamos el comando "land"
                cmd_msg = String()
                cmd_msg.data = "land"
                self.command_pub.publish(cmd_msg)
                
                # Marcamos que el failsafe ha sido activado
                self.failsafe_triggered = True
            else:
                # Si el failsafe ya está activo, solo informamos
                self.get_logger().info(f'Failsafe activo. Esperando aterrizaje... Batería: {battery_level}%', throttle_duration_sec=5.0)


def main(args=None):
    rclpy.init(args=args)
    
    node = BatteryFailsafeNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('Cerrando nodo de seguridad.')
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
