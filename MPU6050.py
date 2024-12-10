# PROYECTO: Falcon ESP
# Código de MPU6050
# Este código obtiene los valores de inclinación y muestra un mensaje según el movimiento
# Enciende LEDs dependiendo de la dirección detectada.

import machine
import time
from mpu6050 import MPU6050

# Pines para LEDs (ajustar según hardware)
led_derecha = machine.Pin(5, machine.Pin.OUT)
led_izquierda = machine.Pin(18, machine.Pin.OUT)
led_adelante = machine.Pin(19, machine.Pin.OUT)
led_atras = machine.Pin(21, machine.Pin.OUT)
led_esq_adelante_der = machine.Pin(22, machine.Pin.OUT)
led_esq_adelante_izq = machine.Pin(23, machine.Pin.OUT)
led_esq_atras_der = machine.Pin(25, machine.Pin.OUT)
led_esq_atras_izq = machine.Pin(26, machine.Pin.OUT)

# Inicializar todos los LEDS apagados
for led in [led_derecha, led_izquierda, led_adelante, led_atras, led_esq_adelante_der, led_esq_adelante_izq, led_esq_atras_der, led_esq_atras_izq]:
    led.value(0)

# Inicializar I2C y MPU6050
i2c = machine.I2C(scl=machine.Pin(22), sda=machine.Pin(21))
sensor = MPU6050(i2c)

while True:
    # Obtener valores de aceleración
    ax, ay, az = sensor.get_accel_data()
    
    # Lógica simple: 
    # si ax > un umbral => derecha
    # si ax < -umbral => izquierda
    # si ay > un umbral => adelante
    # si ay < -umbral => atrás
    # y sus combinaciones para diagonales
    umbral = 0.5
    mov_derecha = ax > umbral
    mov_izquierda = ax < -umbral
    mov_adelante = ay > umbral
    mov_atras = ay < -umbral
    
    # Apagar todos los LEDs
    for led in [led_derecha, led_izquierda, led_adelante, led_atras, led_esq_adelante_der, led_esq_adelante_izq, led_esq_atras_der, led_esq_atras_izq]:
        led.value(0)
    
    if mov_adelante and mov_derecha:
        print("Moviendo en esquina adelante-derecha")
        led_esq_adelante_der.value(1)
    elif mov_adelante and mov_izquierda:
        print("Moviendo en esquina adelante-izquierda")
        led_esq_adelante_izq.value(1)
    elif mov_atras and mov_derecha:
        print("Moviendo en esquina atras-derecha")
        led_esq_atras_der.value(1)
    elif mov_atras and mov_izquierda:
        print("Moviendo en esquina atras-izquierda")
        led_esq_atras_izq.value(1)
    elif mov_derecha:
        print("Moviendo a la derecha")
        led_derecha.value(1)
    elif mov_izquierda:
        print("Moviendo a la izquierda")
        led_izquierda.value(1)
    elif mov_adelante:
        print("Moviendo hacia adelante")
        led_adelante.value(1)
    elif mov_atras:
        print("Moviendo hacia atras")
        led_atras.value(1)
    else:
        print("Estable")

    time.sleep(0.5)
