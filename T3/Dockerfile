# Seleccionar una imagen base
FROM osrf/ros:jazzy-desktop

# Autor
LABEL maintainer=joseph.jaramillo@ucuenca.edu.ec

# Instalar dependencias comunes, la librería serial de Python y herramientas de terminal
# Agregamos python3-serial y minicom/screen para depuración y serial
RUN apt update && apt upgrade -y && \
    apt install -y \
    python3-colcon-common-extensions \
    nano \
    python3-pip \
    python3-matplotlib \
    python3-serial \
    minicom \
    tree
    
# Se ejecuta el script de entrada ROS2 (que carga /opt/ros/jazzy/setup.bash)
ENTRYPOINT ["/ros_entrypoint.sh"]

# Crear workspace
RUN mkdir -p /root/ros2_ws/src

WORKDIR /root/ros2_ws/src

# Crear el paquete ROS2 (usando el nombre 'sensor_serial' del taller)
RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash && \ 
    ros2 pkg create --build-type ament_python sensor_serial --license MIT"
    
# Copiar setup.py al directorio raíz del paquete (sensor_serial/)
WORKDIR /root/ros2_ws/src/sensor_serial
COPY config/setup.py .

# Copiar los archivos de los nodos al subdirectorio del paquete (sensor_serial/sensor_serial/)
WORKDIR /root/ros2_ws/src/sensor_serial/sensor_serial
COPY nodes/*.py .

# Compilar el workspace
WORKDIR /root/ros2_ws
RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash && \
    colcon build"
    
# Automatizamos la carga del entorno local del proyecto al iniciar el shell
RUN echo "source /root/ros2_ws/install/setup.bash" >> ~/.bashrc
    
# Establecemos el directorio de trabajo por defecto
WORKDIR /root/ros2_ws 

# Comando por defecto al arrancar el contenedor: Abrir una shell de Bash
CMD ["bash"]
