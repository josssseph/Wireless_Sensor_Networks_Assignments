import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import matplotlib
matplotlib.use('TkAgg') 

import matplotlib.pyplot as plt
import matplotlib.animation as animation
import collections
import threading
import time 


MAX_DATA_POINTS = 50 

from rclpy.executors import MultiThreadedExecutor


class PlotterMonitorNode(Node):
    """
    Nodo que se suscribe a dos tópicos y actualiza un gráfico.
    """
    def __init__(self):
        super().__init__('plotter_node_temp')
        self.start_time = time.time()
        
        # Variables de datos
        self.tiempos = collections.deque(maxlen=MAX_DATA_POINTS)
        self.temperaturas_1 = collections.deque(maxlen=MAX_DATA_POINTS)
        self.temperaturas_2 = collections.deque(maxlen=MAX_DATA_POINTS)

        # Configurar suscriptores
        self.subscription_1 = self.create_subscription(Float32, 'temperature_1', self.callback_temp1, 10)
        self.subscription_2 = self.create_subscription(Float32, 'temperature_2', self.callback_temp2, 10)
        
        # Inicializar Matplotlib (se ejecuta en el hilo principal)
        plt.ion() # Modo interactivo ON
        self.fig, self.ax = plt.subplots(figsize=(10, 5))
        self.line1, = self.ax.plot([], [], marker='.', linestyle='-', label='Sensor 1 (ID 0x100)')
        self.line2, = self.ax.plot([], [], marker='.', linestyle='-', label='Sensor 2 (ID 0x200)')
        
        self.ax.set_title('Temperaturas CAN en Tiempo Real (°C)')
        self.ax.set_xlabel('Tiempo Relativo (s)')
        self.ax.set_ylabel('Temperatura (°C)')
        self.ax.grid(True)
        self.ax.legend()
        
        # FuncAnimation es la forma preferida de actualizar gráficas sin bloquear
        self.ani = animation.FuncAnimation(self.fig, self.update_plot, interval=100, cache_frame_data=False)
        
        self.get_logger().info('Monitor gráfico iniciado. Ejecutando ROS en hilo secundario...')

    def callback_temp1(self, msg):
        current_time = time.time() - self.start_time
        temp = msg.data
        
        # Añade un nuevo punto de tiempo solo cuando es estrictamente necesario
        if not self.tiempos or current_time > self.tiempos[-1]:
            self.tiempos.append(current_time)
            self.temperaturas_1.append(temp)
            # Rellenar Temp 2 con el último valor conocido si no hay uno nuevo
            self.temperaturas_2.append(self.temperaturas_2[-1] if self.temperaturas_2 else temp) 
        else:
            # Actualiza el último valor si es una actualización rápida
            self.temperaturas_1[-1] = temp
        
        # self.get_logger().info(f'Dato SENSOR 1: {temp:.2f} °C')

    def callback_temp2(self, msg):
        temp = msg.data
        
        # Sincronización de datos: Si el punto de tiempo existe, actualizamos Temp 2
        if self.temperaturas_2:
            self.temperaturas_2[-1] = temp
        else:
            # Si Temp 1 aún no ha llegado, forzamos la creación de un punto
            current_time = time.time() - self.start_time
            self.tiempos.append(current_time)
            self.temperaturas_1.append(temp) 
            self.temperaturas_2.append(temp)
            
        # self.get_logger().info(f'Dato SENSOR 2: {temp:.2f} °C')

    def update_plot(self, frame):
        """ Actualiza la gráfica con los nuevos datos. """
        if not self.tiempos:
            return self.line1, self.line2,

        tiempos_list = list(self.tiempos)
        self.line1.set_data(tiempos_list, list(self.temperaturas_1))
        self.line2.set_data(tiempos_list, list(self.temperaturas_2))
        
        # Auto-escalar
        self.ax.relim()
        self.ax.autoscale_view()
        
        # Asegurar el límite X
        self.ax.set_xlim(tiempos_list[0], tiempos_list[-1] + 1)
        
        # No se necesita fig.canvas.draw() ni flush_events() cuando se usa FuncAnimation con un backend interactivo

        return self.line1, self.line2,


def main(args=None):
    rclpy.init(args=args)
    node = PlotterMonitorNode()
    
    # Usar un MultiThreadedExecutor para permitir que ROS procese callbacks en un hilo secundario
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    # Iniciar la ejecución de ROS en un hilo separado
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    try:
        node.fig.show() # Intenta mostrar la figura inmediatamente
        plt.show(block=False) # Inicia el bucle de eventos sin bloquear
        
        while rclpy.ok():
            plt.pause(0.1)  # Pausa que permite que la GUI de Matplotlib se dibuje
            
    except KeyboardInterrupt:
        node.get_logger().info('Interrupción por teclado recibida')
    except Exception as e:
        node.get_logger().error(f"Error en el bucle principal de la GUI: {e}")
    finally:
        node.get_logger().info('Cerrando monitor y apagando ROS.')
        
        plt.close(node.fig) 
        if executor_thread.is_alive():
            executor.shutdown()
            executor_thread.join()

        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
    
