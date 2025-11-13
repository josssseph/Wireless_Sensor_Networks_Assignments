#!/bin/bash

# Nombres de los contenedores a monitorear
CONTAINER_NAMES="drone_ros_dev grafana_tello influxdb_tello"

# Nombre del archivo de salida
OUTPUT_FILE="docker_metrics.csv"

# Encabezados del CSV
if [ ! -f $OUTPUT_FILE ]; then
    echo "TIMESTAMP,CONTAINER_ID,NAME,CPU_USAGE,MEM_USAGE,MEM_LIMIT,NET_I_O,BLOCK_I_O,PIDS" > $OUTPUT_FILE
fi

# El bucle ejecutará docker stats cada 5 segundos. 
while true; do
    # Captura la marca de tiempo actual
    TIMESTAMP=$(date +%Y-%m-%d\ %H:%M:%S)
    
    # EJECUCIÓN ESPECÍFICA: Se añaden los nombres de los contenedores al final del comando.
    docker stats --no-stream \
                 --format "{{.ID}},{{.Name}},{{.CPUPerc}},{{.MemUsage}},{{.NetIO}},{{.BlockIO}},{{.PIDs}}" \
                 $CONTAINER_NAMES | \
    
    while read LINE; do
        # La columna MemUsage de docker stats ya incluye USAGE / LIMIT (ej. 100MiB / 2GiB)
        # El script de Python anterior maneja esta limpieza.
        
        # Separamos el uso y el límite para insertarlos en el formato CSV correcto
        # Ejemplo de LINE (solo métricas): d91603b65ef5,drone_ros_dev,1.50%,150MiB / 2GiB,12kB / 5kB,0B / 0B,15
        
        # El formato de docker stats --format... no soporta separaciones complejas, 
        # por lo que seguiremos usando la salida como está y dejaremos el parsing más fino a Python, 
        # que es más robusto para manipular strings y unidades de medida.
        
        # La salida generará una sola columna 'MEM_USAGE' con el valor "uso / límite", 
        # que será separado por el script de Python.
        echo "$TIMESTAMP,$LINE" >> $OUTPUT_FILE
    done
    
    # Espera 5 segundos antes de la siguiente muestra
    sleep 5
done
