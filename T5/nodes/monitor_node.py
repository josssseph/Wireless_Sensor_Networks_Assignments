import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class DualMonitorNode(Node):
    def __init__(self):
        super().__init__('monitor_node')
        
        # Suscripción para el Sensor 1 (ID 0x100)
        self.subscription_1 = self.create_subscription(
            Float32,
            'temperature_1',
            self.callback_sensor_1,
            10
        )
        # Suscripción para el Sensor 2 (ID 0x200)
        self.subscription_2 = self.create_subscription(
            Float32,
            'temperature_2',
            self.callback_sensor_2,
            10
        )
        
        self.get_logger().info('Nodo de monitor dual inicializado - Esperando datos de ambos sensores...')

    def callback_sensor_1(self, msg):
        """Callback para los datos del Sensor 1."""
        self.get_logger().info(f'-SENSOR 1 (ID 0x100): {msg.data:.2f} °C')

    def callback_sensor_2(self, msg):
        """Callback para los datos del Sensor 2."""
        self.get_logger().info(f'>SENSOR 2 (ID 0x200): {msg.data:.2f} °C')

def main(args=None):
    rclpy.init(args=args)
    
    node = None
    try:
        node = DualMonitorNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node:
             node.get_logger().info('Interrupción por teclado recibida')
    finally:
        if node:
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
