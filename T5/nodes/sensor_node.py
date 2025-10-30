import rclpy
from rclpy.node import Node
import can
from std_msgs.msg import Float32

class CANListener(Node):
    """
    Nodo que se suscribe a la interfaz CAN ('can0') para leer
    mensajes con ID 0x100 y extraer la temperatura en formato de 16 bits.
    """
    def __init__(self):
        super().__init__('can_listener')
        
        # Publica la temperatura en un tópico de Float32
        self.publisher_ = self.create_publisher(Float32, 'temperature', 10)
        
        try:
            # Inicializa la interfaz CAN (usa 'can0' ya que se usa --net=host)
            self.bus = can.interface.Bus(channel='can0', bustype='socketcan')
            self.get_logger().info('✅ Listening on CAN interface can0...')
            
            # Crea un temporizador para chequear el bus CAN periódicamente
            self.timer = self.create_timer(0.1, self.read_can) # 10 veces por segundo
            
        except Exception as e:
            self.get_logger().error(f'❌ Error initializing CAN bus: {e}')
            self.bus = None # Deshabilita la lectura si falla la conexión

    def read_can(self):
        """
        Intenta recibir un mensaje CAN y procesarlo.
        """
        if not self.bus:
            return

        msg = self.bus.recv(timeout=0.01) # Espera 10ms por un mensaje
        
        # Verifica el ID de arbitraje y la longitud de los datos
        if msg and msg.arbitration_id == 0x100 and len(msg.data) >= 2:
            
            # Decodificación de 16 bits (Little Endian o Big Endian, asumiendo BE para temperatura)
            # t10 = (msg.data[0] << 8) | msg.data[1]  # Valor de 16 bits
            
            # Usando la decodificación de la guía
            t10 = (msg.data[0] << 8) | msg.data[1]
            
            # Manejo de valor con signo (signed value) si el bit 15 está activo (0x8000)
            if t10 & 0x8000:
                t10 -= 65536
                
            # Escala a grados Celsius (dividiendo por 10.0)
            tempC = t10 / 10.0
            
            # Publica el mensaje
            temp_msg = Float32(data=tempC)
            self.publisher_.publish(temp_msg)
            
            self.get_logger().info(f'Publicando en el tópico temperature: {tempC:.1f} C')

def main(args=None):
    rclpy.init(args=args)
    node = CANListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
