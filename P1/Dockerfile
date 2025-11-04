# Proyecto 1
# Seleccionar una imagen base
FROM osrf/ros:jazzy-desktop

# Autor
LABEL maintainer=joseph.jaramillo@ucuenca.edu.ec

RUN apt update && apt upgrade -y && \
    apt install -y \
    python3-colcon-common-extensions \
    nano \
    python3-pip \
    tree \
    net-tools \
    python3-opencv \
    ros-jazzy-cv-bridge \
    netcat-openbsd

# Instalamos las dependencias de djitellopy que apt NO nos dio
RUN pip3 install av pillow --break-system-packages

# Instalamos djitellopy SIN sus dependencias (para evitar el conflicto de opencv)
RUN pip3 install djitellopy --no-dependencies --break-system-packages


# Se ejecuta el script de entrada ROS2 (que carga /opt/ros/jazzy/setup.bash)
ENTRYPOINT ["/ros_entrypoint.sh"]

# Crear workspace
RUN mkdir -p /root/ros2_ws/src

WORKDIR /root/ros2_ws/src

# Crear el paquete ROS2 (usando el nombre 'drone_pkg') #sensor_serial
RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash && \ 
    ros2 pkg create --build-type ament_python drone_pkg --license MIT"
    
# Copiar setup.py al directorio raíz del paquete (sensor_serial/)
WORKDIR /root/ros2_ws/src/drone_pkg
COPY config/setup.py .

# Copiar los archivos de los nodos al subdirectorio del paquete (sensor_serial/sensor_serial/)
WORKDIR /root/ros2_ws/src/drone_pkg/drone_pkg
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
