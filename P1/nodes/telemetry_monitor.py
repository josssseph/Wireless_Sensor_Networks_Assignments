#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
# Importaciones de InfluxDB
from influxdb_client import InfluxDBClient, Point, WritePrecision
from influxdb_client.client.write_api import SYNCHRONOUS
import sys
import os # Necesario para limpiar la consola

# --- CONFIGURACIÓN DE INFLUXDB ---
# ¡Asegúrate de cambiar INFLUX_TOKEN por tu valor real!
INFLUX_URL = "http://localhost:8086"
INFLUX_TOKEN = "mi-token-secreto"
INFLUX_ORG = "ParrotOrg"
INFLUX_BUCKET = "tello_data"
# ----------------------------------

class TelemetryBridgeNode(Node):
    """
    Nodo Unificado: Lee telemetría de ROS 2, la escribe en InfluxDB y la muestra en terminal.
    """
    def __init__(self):
        super().__init__('telemetry_bridge')
        self.get_logger().info('Iniciando el nodo puente ROS <-> InfluxDB con monitor de telemetría.')
        
        # 1. Almacenamiento Local (Para el Dashboard)
        self.latest_battery = -1
        self.latest_speed_x = 0
        self.latest_height = 0

        # 2. Conexión InfluxDB
        self.influx_client = None
        self.write_api = None
        try:
            self.influx_client = InfluxDBClient(url=INFLUX_URL, token=INFLUX_TOKEN, org=INFLUX_ORG)
            self.write_api = self.influx_client.write_api(write_options=SYNCHRONOUS)
            self.get_logger().info(f'Conexión a InfluxDB en {INFLUX_URL} establecida.')
        except Exception as e:
            self.get_logger().error(f'Error al conectar con InfluxDB: {e}. Desactivando funcionalidad de DB.')
        
        # 3. Suscriptores ROS 2
        # Todos los suscriptores llaman a la función unificada 'process_telemetry'
        self.battery_sub = self.create_subscription(
            Int32, '/tello/battery', 
            lambda msg: self.process_telemetry(msg, "battery_level", "percent"), 
            10
        )
        self.speed_sub = self.create_subscription(
            Int32, '/tello/speed_x', 
            lambda msg: self.process_telemetry(msg, "speed_x", "cm_per_second"), 
            10
        )
        self.height_sub = self.create_subscription(
            Int32, '/tello/height', 
            lambda msg: self.process_telemetry(msg, "drone_height", "cm"), 
            10
        )
        
        # 4. Timer para el Dashboard (Actualiza 2 veces/segundo)
        self.print_timer = self.create_timer(0.5, self.print_dashboard) 

    
    def process_telemetry(self, msg, field_name: str, units: str):
        """
        Función Callback Unificada: Almacena localmente y escribe en InfluxDB.
        """
        value = msg.data

        # 1. Almacenar localmente (para el dashboard)
        if field_name == "battery_level":
            self.latest_battery = value
        elif field_name == "speed_x":
            self.latest_speed_x = value
        elif field_name == "drone_height":
            self.latest_height = value

        # 2. Escribir en InfluxDB (si la conexión es válida)
        if self.write_api:
            try:
                point = (
                    Point("tello_telemetry")
                    .tag("topic", field_name)
                    .tag("units", units)
                    .field("value", value)
                    .time(None, write_precision=WritePrecision.MS)
                )
                self.write_api.write(bucket=INFLUX_BUCKET, record=point)
            except Exception as e:
                # Nota: Estos errores de escritura son normales si la DB está saturada o la red es lenta.
                # No detenemos el nodo, solo avisamos.
                self.get_logger().debug(f'Error al escribir el punto "{field_name}": {e}')
    
    def print_dashboard(self):
        """
        Limpia la consola y muestra los últimos datos de telemetría.
        """
        # Limpiar la consola (funciona en Linux/macOS/Windows)
        os.system('cls' if os.name == 'nt' else 'clear')
        
        self.get_logger().info('--- MONITOR DE TELEMETRÍA TELLO ---')
        self.get_logger().info(f'      **Batería** : {self.latest_battery} %')
        self.get_logger().info(f'      **Altura** : {self.latest_height} cm')
        self.get_logger().info(f'  **Velocidad (X)** : {self.latest_speed_x} cm/s')
        self.get_logger().info('-------------------------------------')
        if not self.write_api:
            self.get_logger().info('🚨 **MODO OFFLINE:** Falló la conexión a InfluxDB. Solo monitorizando.')
        self.get_logger().info('(Presiona Ctrl+C para salir)')


    def destroy_node(self):
        """Cierra la conexión de InfluxDB al detener el nodo."""
        if self.influx_client:
            self.influx_client.close()
            self.get_logger().info('Conexión a InfluxDB cerrada.')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    node = None
    try:
        node = TelemetryBridgeNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        if node:
            node.get_logger().error(f"Error fatal: {e}")
    finally:
        if node:
            node.destroy_node()
        print("\nMonitor de telemetría detenido.")
        rclpy.shutdown()

if __name__ == '__main__':
    main()
