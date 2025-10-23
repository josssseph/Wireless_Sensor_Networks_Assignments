import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Float32

class ProcessorNode(Node):
    def __init__(self):
        super().__init__('processor_node')
        
        # Crear suscripción al topic 'sensor_data'
        self.subscription = self.create_subscription(
            Int32, 
            'sensor_data', 
            self.listener_callback, 
            10
        )
        
        # Crear publisher para el topic 'temperature_celsius'
        self.publisher_ = self.create_publisher(
            Float32, 
            'temperature_celsius', 
            10
        )
        
        self.get_logger().info('Processor node initialized')

    def listener_callback(self, msg):
        raw_value = msg.data
        
        # Convertir valor crudo a temperatura (escala 0-100°C)
        temperature = (raw_value / 1023.0) * 100.0
        
        temp_msg = Float32()
        temp_msg.data = temperature
        
        # Publicar temperatura procesada
        self.publisher_.publish(temp_msg)
        
        self.get_logger().info(f'Temperatura procesada: {temperature:.2f} °C')

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = ProcessorNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Interrupción por teclado recibida')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
