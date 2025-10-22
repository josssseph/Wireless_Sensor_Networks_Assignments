# Nodo Plotter

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import matplotlib.pyplot as plt
import os


class PlotterNode(Node):
    def __init__(self):
        super().__init__('plotter_node')
        self.subscription = self.create_subscription(
            String,
            'sensor_data',
            self.listener_callback,
            10
        )
        self.subscription

        # Variables para almacenar los datos
        self.temperaturas = []
        self.tiempos_muestra = 0 # Usaremos un contador para el eje X
        self.muestras_x = []
        
	# Contador para el nombre del archivo
        self.plot_counter = 0 
        # ----------------------------------------------------

        # Timer para generar el gráfico cada 5 segundos
        self.timer = self.create_timer(5.0, self.generate_plot)

        # La base de la ruta de salida (directorio)
        self.output_dir = '/root/ros2_ws/data'
        
        # Crear carpeta si no existe
        os.makedirs(self.output_dir, exist_ok=True)

        self.get_logger().info('Nodo plotter iniciado. Esperando datos del sensor...')

    def listener_callback(self, msg):
        """Recibe los mensajes del tópico sensor_data"""
        try:
            # Extrae el valor numérico (asumiendo el formato "Temperatura: 25 C")
            temperatura = int(''.join(filter(str.isdigit, msg.data)))
            
            self.temperaturas.append(temperatura)
            
            # 📈 Registra la muestra
            self.tiempos_muestra += 1 
            self.muestras_x.append(self.tiempos_muestra)

            self.get_logger().info(f'Dato recibido: {temperatura} °C (Muestra #{self.tiempos_muestra})')
        except ValueError:
            self.get_logger().warn(f'Mensaje no válido recibido: {msg.data}')

    def generate_plot(self):
        """Genera y guarda el gráfico cada 5 segundos"""
        if not self.temperaturas:
            self.get_logger().warn('Aún no hay datos para graficar.')
            return
            
        # Incrementar contador y crear nombre de archivo único
        self.plot_counter += 1
        filename = f'sensor_plot_{self.plot_counter}.png'
        # Construir la ruta completa
        self.output_path_unique = os.path.join(self.output_dir, filename)
        # --------------------------------------------------------------------

        plt.figure(figsize=(8, 4))
        plt.plot(self.muestras_x, self.temperaturas, marker='o', linestyle='-', label='Temperatura')
        plt.title(f'Temperatura del Sensor (Gráfico #{self.plot_counter})') # Se actualiza el título
        plt.xlabel('Número de Muestra') 
        plt.ylabel('Temperatura (°C)')
        
        # Limita el número de muestras si el gráfico se vuelve muy grande
        if len(self.muestras_x) > 15:
            plt.xticks(self.muestras_x[-15::2])
        else:
            plt.xticks(self.muestras_x)
            
        plt.grid(True)
        plt.legend()
        plt.tight_layout()

        # Usar la nueva ruta única
        plt.savefig(self.output_path_unique)
        plt.close()

        self.get_logger().info(f'Gráfico actualizado y guardado en {self.output_path_unique}')


def main(args=None):
    rclpy.init(args=args)
    node = PlotterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass # Permite una salida limpia
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
