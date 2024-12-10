# PROYECTO: Falcon ESP
# Código del Joystick

import machine
import time

adc_x = machine.ADC(machine.Pin(34))
adc_y = machine.ADC(machine.Pin(35))

#valor medio ~2048 en un ESP32 (0-4095)
centro = 2048
umbral = 300

while True:
    x_val = adc_x.read()
    y_val = adc_y.read()
    
    dx = x_val - centro
    dy = y_val - centro
    
    if dx > umbral:
        print("Joystick derecha")
    elif dx < -umbral:
        print("Joystick izquierda")
        
    if dy > umbral:
        print("Joystick arriba")
    elif dy < -umbral:
        print("Joystick abajo")

    time.sleep(0.5)
