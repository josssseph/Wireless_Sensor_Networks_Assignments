import rclpy
from rclpy.node import Node
import can
from std_msgs.msg import Float32

class CANListenerReto(Node):
    """
    Nodo que se suscribe a la interfaz CAN ('can0') para leer
    mensajes con IDs 0x100 y 0x200 y publicar en dos tópicos separados.
    """
    def __init__(self):
        super().__init__('sensor_node')
        
        # Publicadores para el Reto con dos sensores
        self.publisher_1 = self.create_publisher(Float32, 'temperature_1', 10)
        self.publisher_2 = self.create_publisher(Float32, 'temperature_2', 10)
        
        try:
            # Inicializa la interfaz CAN (usa 'can0' ya que se tiene acceso al red del host)
            self.bus = can.interface.Bus(channel='can0', bustype='socketcan')
            self.get_logger().info('Listening on CAN interface can0 for IDs 0x100 and 0x200...')
            
            # Crea un temporizador para chequear el bus CAN periódicamente
            self.timer = self.create_timer(0.1, self.read_can) # 10 veces por segundo
            
        except Exception as e:
            self.get_logger().error(f'Error initializing CAN bus. Check if "sudo ip link set can0 up..." was run: {e}')
            self.bus = None # Deshabilita la lectura si falla la conexión

    def decode_temperature(self, data):
        """Función auxiliar para decodificar los 2 bytes de temperatura (16 bits)."""
        # Se asume la decodificación de la guía: 16 bits * 10 = Temperatura
        
        t10 = (data[0] << 8) | data[1]
        
        # Manejo de valor con signo (signed value)
        if t10 & 0x8000: # Si el bit 15 (signo) está activo
            t10 -= 65536 # Restar 2^16 para obtener el valor negativo correcto
            
        # Escala a grados Celsius (dividiendo por 10.0)
        tempC = t10 / 10.0
        return tempC

    def read_can(self):
        """
        Intenta recibir un mensaje CAN, determinar su ID y publicarlo en el tópico correcto.
        """
        if not self.bus:
            return

        # Solo intenta leer un mensaje; si hay más, se leerán en el siguiente ciclo del timer
        msg = self.bus.recv(timeout=0.0) # No esperar, solo chequear si hay algo
        
        if msg and len(msg.data) >= 2:
            
            tempC = self.decode_temperature(msg.data)
            temp_msg = Float32(data=tempC)
            
            # --- Lógica de Selección de Tópico---
            if msg.arbitration_id == 0x100:
                self.publisher_1.publish(temp_msg)
                self.get_logger().info(f'SENSOR 1 (ID 0x100) -> {tempC:.1f} C')
            
            elif msg.arbitration_id == 0x200:
                self.publisher_2.publish(temp_msg)
                self.get_logger().info(f'SENSOR 2 (ID 0x200) -> {tempC:.1f} C')
                pass


def main(args=None):
    rclpy.init(args=args)
    node = CANListenerReto()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.bus:
            # Cerrar el bus al apagar el nodo
            node.bus.shutdown() 
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
