import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import serial

class SensorNode(Node):
    def __init__(self):
        super().__init__('sensor_node')
        
        # Configuración del puerto serial para Arduino
        self.ser = serial.Serial('/dev/ttyUSB1', 115200, timeout=1)
        
        # Crear publisher para el topic 'sensor_data'
        self.publisher_ = self.create_publisher(Int32, 'sensor_data', 10)
        
        # Crear timer para publicar datos cada 1 segundo
        self.timer = self.create_timer(1.0, self.publish_data)
        
        self.get_logger().info('Sensor node initialized')

    def publish_data(self):
        try:
            # Leer línea del puerto serial
            line = self.ser.readline().decode('utf-8').strip()
            
            if line.isdigit():  # Asegurar que solo son números
                value = int(line)
                msg = Int32()
                msg.data = value
                self.publisher_.publish(msg)
                self.get_logger().info(f'Publicando dato: {value}')
            else:
                self.get_logger().warning(f'Dato inválido: "{line}"')
                
        except Exception as e:
            self.get_logger().error(f'Error al leer del puerto serial: {e}')

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = SensorNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
