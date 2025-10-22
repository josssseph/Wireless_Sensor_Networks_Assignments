# Seleccionar una imagen base
FROM osrf/ros:jazzy-desktop

# Autor
LABEL maintainer=joseph.jaramillo@ucuenca.edu.ec

# Instalar dependencias
RUN apt update && apt upgrade -y && \
    apt install -y \
    python3-colcon-common-extensions \
    nano \
    python3-pip \
    python3-matplotlib \
    tree
    
# Se ejecuta el script de entrada ROS2 (que carga /opt/ros/jazzy/setup.bash)
# Se ejecuta cuando arrancamos el contenedor
ENTRYPOINT ["/ros_entrypoint.sh"]

# Crear workspace
RUN mkdir -p /root/ros2_ws/src

WORKDIR /root/ros2_ws/src

# Crear el paquete ROS2 (esto crea la estructura automáticamente)
# La primera línea se ejecuta durante el proceso de construcción de la imagen
RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash && \ 
    ros2 pkg create --build-type ament_python sensor_program --license MIT"

WORKDIR /root/ros2_ws/src/sensor_program
COPY config/setup.py .

WORKDIR /root/ros2_ws/src/sensor_program/sensor_program
COPY nodes/*.py .

WORKDIR /root/ros2_ws

RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash && \
    colcon build"
    
# Automatizamos la carga del entorno local del proyecto al iniciar la shell
RUN echo "source /root/ros2_ws/install/setup.bash" >> ~/.bashrc

# Establecemos el directorio de trabajo por defecto (para que la shell se abra aquí)
WORKDIR /root/ros2_ws 

# Comando por defecto al arrancar el contenedor: Abrir una shell de Bash
# Esto se convierte en el argumento del ENTRYPOINT /ros_entrypoint.sh
CMD ["bash"]
