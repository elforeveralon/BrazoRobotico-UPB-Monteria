# BrazoRobotico-UPB-Monteria

Proyecto de grado desarrollado para el programa de Ingeniería Electrónica de la Universidad Pontificia Bolivariana - Seccional Montería.

## Archivos principales

### proy.py
Implementa el modo manual de operación mediante mando USB, incluyendo monitoreo energético mediante el sensor INA226.

### mover_brazo.py
Implementa el modo automático de detección y manipulación de objetos mediante visión artificial y cinemática inversa.

### verificar_homografia.py
Permite validar la transformación geométrica utilizada para convertir coordenadas de imagen a coordenadas físicas del área de trabajo.

### homografia.npy
Archivo de calibración que almacena la matriz de homografía utilizada por el subsistema de visión artificial.

## Requisitos

- Raspberry Pi 5
- Python 3
- OpenCV
- NumPy
- Picamera2
- gpiozero
- smbus2
