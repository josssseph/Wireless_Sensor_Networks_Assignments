import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import matplotlib
matplotlib.use('TkAgg') 

import matplotlib.pyplot as plt
import matplotlib.animation as animation
import collections
import threading 

# --- Configuración para la gráfica dinámica ---
# Definir la longitud máxima de datos a mostrar
MAX_DATA_POINTS = 15

class PlotterNode(Node):
    """
    Nodo que se suscribe al tópico de temperatura final y actualiza 
    un gráfico dinámico en tiempo real.
    """
    def __init__(self):
        super().__init__('plotter_node_temp')
        
        # Suscripción al tópico y tipo de mensaje correctos
        self.subscription = self.create_subscription(
            Float32,
            'temperature', 
            self.listener_callback,
            10
        )
        self.subscription

        # Variables para almacenar los datos (usando deque para limitar el tamaño)
        self.temperaturas = collections.deque(maxlen=MAX_DATA_POINTS)
        self.tiempos_muestra = 0
        self.muestras_x = collections.deque(maxlen=MAX_DATA_POINTS)
        
        # ----------------- CONFIGURACIÓN DE MATPLOTLIB -----------------
        # 1. Configurar modo interactivo
        plt.ion()
        
        # 2. Crear la figura y el eje
        self.fig, self.ax = plt.subplots(figsize=(8, 4))
        self.line, = self.ax.plot([], [], marker='o', linestyle='-', label='Temperatura')
        
        # 3. Configurar límites y etiquetas
        self.ax.set_title('Temperatura en Tiempo Real (°C)')
        self.ax.set_xlabel('Número de Muestra')
        self.ax.set_ylabel('Temperatura (°C)')
        self.ax.grid(True)
        self.ax.legend()
        self.ax.set_autoscale_on(True)
        
        # 4. Crear el objeto de animación que llama a update_plot
        # Intervalo de 100 ms (10 FPS) para la actualización de la gráfica
        self.ani = animation.FuncAnimation(self.fig, self.update_plot, interval=100)
        
        # Asegurarse de que el thread de Matplotlib se ejecuta
        plt.show(block=False) 
        # -----------------------------------------------------------------

        self.get_logger().info('Nodo plotter dinámico iniciado. Esperando datos del sensor...')

    def listener_callback(self, msg):
        """
        Recibe los mensajes del tópico 'temperatura_celsius' (Float32).
        """
        temperature = msg.data 
        
        # Añadir datos a los deques
        self.temperaturas.append(temperature)
        self.tiempos_muestra += 1 
        self.muestras_x.append(self.tiempos_muestra)

        self.get_logger().info(f'Dato recibido: {temperature:.2f} °C (Muestra #{self.tiempos_muestra})')

    def update_plot(self, frame):
        """
        Función callback llamada por FuncAnimation para actualizar la gráfica.
        """
        if not self.temperaturas:
            return self.line, # No actualizar si no hay datos

        # Actualizar los datos de la línea
        self.line.set_data(list(self.muestras_x), list(self.temperaturas))
        
        # Auto-escalar los ejes para que la gráfica se ajuste a los nuevos datos
        self.ax.relim()      # Recalcula los límites de los datos
        self.ax.autoscale_view() # Aplica los nuevos límites de los datos
        
        # Limpiar el eje X para mostrar solo las últimas 50 muestras
        self.ax.set_xlim(max(0, self.tiempos_muestra - MAX_DATA_POINTS), self.tiempos_muestra)
        
        # Dibujar la figura
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
        
        return self.line,


def main(args=None):
    rclpy.init(args=args)
    node = PlotterNode()
    
    # 🚨 NOTA IMPORTANTE: Para que FuncAnimation funcione dentro de ROS (que usa su propio
    # bucle), se usa FuncAnimation y plt.show(block=False). La función rclpy.spin(node)
    # debe ser capaz de coexistir con el bucle de eventos de Matplotlib, lo cual 
    # FuncAnimation ayuda a gestionar.
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        plt.close(node.fig) # Cerrar la figura al salir
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
