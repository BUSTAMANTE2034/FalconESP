# PROYECTO: Falcon ESP
# Código de Ultrasonico

import machine
import time

trig = machine.Pin(13, machine.Pin.OUT)
echo = machine.Pin(12, machine.Pin.IN)

def medir_distancia():
    # Mandar pulso
    trig.value(0)
    time.sleep_us(5)
    trig.value(1)
    time.sleep_us(10)
    trig.value(0)
    
    # Esperar pulso de respuesta
    while echo.value() == 0:
        start = time.ticks_us()
    while echo.value() == 1:
        end = time.ticks_us()
        
    duracion = end - start
    distancia = (duracion / 2) / 29.1  # en cm (aproximado)
    return distancia

while True:
    dist = medir_distancia()
    print("Distancia:", dist, "cm")
    time.sleep(1)
