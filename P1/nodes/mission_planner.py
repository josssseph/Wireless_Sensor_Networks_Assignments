#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String    # Para publicar comandos
from std_msgs.msg import Bool      # Para suscribirse al estado de seguridad
from std_msgs.msg import Int32     # Para suscribirse a la altura

class MissionPlannerNode(Node):
    """
    Nodo 5: Planificador de Misión.
    """
    
    def __init__(self):
        super().__init__('mission_planner')
        self.get_logger().info('Iniciando el planificador de misión.')

        # Estado de la misión
        self.mission_started = False
        self.is_safe_to_fly = False
        self.current_height = 0
        self.mission_step = 0
        self.mission_timer = None # Para controlar la secuencia de pasos

        # Secuencia de la misión (basada en el PDF)
        # NOTA: Los comandos de acción DEBEN ser seguidos por un 'delay_' o 'wait_height'
        self.mission_queue = [
            'takeoff',        # 1. Despegue (Acción)
            'delay_6',        # 2. Pausa 7s (Tiempo para subir/estabilizar)
            'wait_height',    # 3. Esperar a alcanzar 50cm (Condición)
            'delay_3',        # 4. Pausa 3s para estabilizar
            'forward 50',     # 5. Avanzar 50cm (Acción)
            'delay_5',        # 6. Pausa 5s (Tiempo para completar el movimiento)
            'back 50',        # 7. Retroceder 50cm (Acción)
            'delay_5',        # 8. Pausa 5s
            'land'            # 9. Aterrizar (Acción)
        ]

        # Publicador y Suscriptores (Sin cambios, son correctos)
        self.command_pub = self.create_publisher(String, '/tello/command', 10)
        self.safety_sub = self.create_subscription(
            Bool, '/tello/safety_status', self.safety_callback, 10
        )
        self.height_sub = self.create_subscription(
            Int32, '/tello/height', self.height_callback, 10
        )
        
        self.get_logger().info('Esperando señal de seguridad (batería OK) del Nodo 4...')

    # --------------------------------------------------
    # --- CALLBACKS DE SUSCRIPCIÓN ---
    # --------------------------------------------------

    def safety_callback(self, msg):
        """Callback del estado de seguridad (Nodo 4)."""
        self.is_safe_to_fly = msg.data
        
        # Si estamos seguros Y la misión no ha comenzado, la iniciamos.
        if self.is_safe_to_fly and not self.mission_started:
            self.get_logger().info('¡Batería OK! Iniciando misión en 3 segundos...')
            self.mission_started = True
            # Damos un pequeño retraso antes de despegar
            self.create_timer(3.0, self.start_mission)
        elif not self.is_safe_to_fly and self.mission_started:
             # Si ya está volando y la batería baja, el failsafe debería tomar el control.
             # Solo logueamos que el failsafe está activo, si es necesario.
             self.get_logger().warn('¡Failsafe activo! Misión suspendida.', throttle_duration_sec=5.0)
        elif not self.is_safe_to_fly and not self.mission_started:
             self.get_logger().warn('En espera. Batería baja detectada.', throttle_duration_sec=10.0)

    def height_callback(self, msg):
        """Callback de la altura (Nodo 1)."""
        self.current_height = msg.data

    # --------------------------------------------------
    # --- MÁQUINA DE ESTADOS (Misión) ---
    # --------------------------------------------------

    def start_mission(self):
        """Inicia la ejecución del primer paso de la misión."""
        self.get_logger().info('Comenzando ejecución de la misión.')
        self.execute_next_step()

    def execute_next_step(self):
        """
        Ejecuta el paso actual de la misión y avanza al siguiente.
        """
        if self.mission_step >= len(self.mission_queue):
            self.get_logger().info('¡Misión completada!')
            self.mission_started = False # Resetear la bandera
            return

        # Obtenemos el comando actual
        command = self.mission_queue[self.mission_step]
        self.get_logger().info(f'Ejecutando Paso {self.mission_step + 1}/{len(self.mission_queue)}: "{command}"')

        # Avanzamos inmediatamente el puntero *antes* de ejecutar la lógica de pausa.
        self.mission_step += 1 

        if command.startswith('delay_'):
            # Es un comando de pausa (ej. "delay_3")
            delay_time = int(command.split('_')[1])
            self.get_logger().info(f'PAUSA: Esperando {delay_time} segundos...')
            # Creamos un timer que llamará a esta MISMA función después del retraso
            self.mission_timer = self.create_timer(float(delay_time), self.execute_next_step)
            
        elif command == 'wait_height':
            # Es un comando de espera de altura
            self.get_logger().info('ESPERA: Activando verificación de altura (50cm)...')
            # Creamos un timer que verificará la altura cada segundo
            self.height_check_timer = self.create_timer(1.0, self.wait_for_height_callback)

        else:
            # 🛑 CLAVE CORREGIDA: Es un comando de Tello (ej. "takeoff", "forward 50")
            # Enviamos el comando y NO llamamos a execute_next_step() de inmediato.
            # Se ASUME que el siguiente paso de la cola es un 'delay_' o 'wait_'
            # que será ejecutado por el siguiente ciclo o timer.

            cmd_msg = String()
            cmd_msg.data = command
            self.command_pub.publish(cmd_msg)
            self.get_logger().info(f'Comando ROS "{command}" publicado en /tello/command.')

            # **IMPORTANTE:** Aquí NO se llama a execute_next_step(). El siguiente paso
            # DEBE ser un 'delay' o 'wait' que es ejecutado por un timer que ya está en marcha.


    def wait_for_height_callback(self):
        """Verifica si hemos alcanzado la altura deseada (50cm)."""
        # La altura objetivo es 50cm.
        TARGET_HEIGHT = 50

        if self.current_height >= TARGET_HEIGHT:
            self.get_logger().info(f'Altura objetivo alcanzada ({self.current_height}cm). Continuando misión.')
            self.height_check_timer.cancel() # Detenemos este timer de verificación
            self.execute_next_step() # Continuamos con el siguiente paso (debe ser un delay)
        else:
            self.get_logger().info(f'ESPERANDO ALTURA: Actual: {self.current_height}cm (Necesario: {TARGET_HEIGHT}cm)')


def main(args=None):
    rclpy.init(args=args)
    
    node = MissionPlannerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('Cerrando planificador de misión.')
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
