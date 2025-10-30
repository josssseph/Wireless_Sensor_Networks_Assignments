import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class MonitorNode(Node):
    def __init__(self):
        super().__init__('monitor_node')
        
        # Crear suscripción al topic 'temperature_celsius'
        self.subscription = self.create_subscription(
            Float32,
            'temperature',
            self.listener_callback,
            10
        )
        
        self.get_logger().info('Nodo Monitor Inicializado - Esperando datos de temperatura...')

    def listener_callback(self, msg):
        # Mostrar temperatura actual recibida
        self.get_logger().info(f'Temperatura actual: {msg.data:.2f} °C')

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = MonitorNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Interrupción por teclado recibida')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
