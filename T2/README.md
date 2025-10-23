🤖 Taller 2: Introducción a ROS 2 Jazzy y Red de Sensores

Este repositorio contiene el desarrollo del Taller de Introducción a ROS 2 (Robot Operating System 2), utilizando la distribución Jazzy Jalisco y la virtualización con Docker para simular una red de sensores (Wireless Sensor Network - WSN).

El proyecto implementa una arquitectura distribuida de nodos (Publisher/Subscriber) y un nodo de visualización de datos (plotter_node) que utiliza Matplotlib para graficar la temperatura en tiempo real.

🚀 Estructura del Proyecto

El proyecto se organiza en un workspace de ROS 2 llamado ros2_ws dentro del contenedor. Los archivos fuente se separan en carpetas en el host para facilitar la construcción de la imagen:

T2/
├── Dockerfile              # Automatiza el ambiente, dependencias y compilación.
├── config/
│   └── setup.py          # Configuración del paquete, registra todos los nodos.
├── nodes/
│   ├── sensor_node.py      # PUBLICADOR: Genera datos de temperatura aleatorios.
│   ├── reader_node.py      # SUSCRIPTOR: Imprime datos recibidos.
│   ├── reader_node_2.py    # SUSCRIPTOR 2: Imprime datos recibidos (prueba de escalabilidad).
│   └── plotter_node.py     # SUSCRIPTOR/GRAFICADOR: Genera gráficos .png.
├── pcap/                   # Carpeta para almacenar capturas de tráfico de red.
└── data/                   # Carpeta COMPARTIDA para almacenar los gráficos .png generados.

⚙️ Requisitos

    Docker: Necesario para construir y ejecutar el contenedor.

    Git: Para clonar el repositorio.

🛠️ Flujo de Trabajo y Ejecución

El ambiente de desarrollo está totalmente automatizado a través del Dockerfile para garantizar que todos los nodos compilen y que el entorno de ROS 2 esté cargado en cada shell.

1. Construir la Imagen de Docker

Desde el directorio raíz de este repositorio (T2/), construye la imagen.
Bash

docker build -t ros2_sensor_net .

2. Ejecutar el Contenedor con Bind Mounts

Para habilitar el desarrollo dinámico y la persistencia de datos (gráficos), el contenedor se ejecuta con dos Bind Mounts. Esto permite editar los nodos en el host y que los gráficos aparezcan en la carpeta local data/.

Nota: La carpeta data/ debe existir en el host antes de la ejecución.
Bash

# Comando de ejecución con doble Bind Mount:
docker run -it --name ros2_ws_data \
  -v "$(pwd)/nodes":/root/ros2_ws/src/sensor_program/sensor_program \
  -v "$(pwd)/data":/root/ros2_ws/data \
  ros2_sensor_net bash

3. Ejecutar los Nodos (Comunicación Distribuida)

Una vez que el contenedor arranca, el ambiente de ROS 2 ya está cargado automáticamente. Se requieren múltiples terminales para ejecutar la red completa:
Terminal	Rol del Nodo	Comando
Terminal 1 (Actual)	Publicador (Sensor)	ros2 run sensor_program sensor_node
Terminal 2 (Nueva)	Suscriptor (Lector 1)	docker exec -it ros2_ws_data bash → ros2 run sensor_program reader_node
Terminal 3 (Nueva)	Graficador	docker exec -it ros2_ws_data bash → ros2 run sensor_program plotter_node

✨ Actividad Reto: Generación de Gráficos

El nodo plotter_node.py implementa las siguientes características del reto:

    Se suscribe al tópico sensor_data.

    Utiliza un rclpy.timer para activar la función de graficación cada 5 segundos.

    Utiliza Matplotlib para generar el gráfico de Temperatura vs. Muestras.

    Guarda la imagen en la ruta absoluta /root/ros2_ws/data/ con un nombre único (sensor_plot_N.png), asegurando que el historial de gráficas persista en la carpeta local data/ del host.

🔬 Análisis de Tráfico de Red (Opcional)

Se utilizó la imagen nicolaka/netshoot para capturar el tráfico entre contenedores en una red definida por el usuario. El análisis de los paquetes reveló:

    Tráfico RTPS (Real-Time Publish-Subscribe): Comunicación a través de UDP en el puerto 7400, conteniendo mensajes de Descubrimiento de Participantes que permiten a los nodos encontrarse sin un ROS Master central.

    Tráfico IGMP: Mensajes de gestión de multicast necesarios para que el middleware DDS pueda operar de forma eficiente.

Este análisis valida la arquitectura de comunicación distribuida de ROS 2 (DDS/RTPS).
