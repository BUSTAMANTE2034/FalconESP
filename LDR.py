# PROYECTO: Falcon ESP
# Código del LDR
# Lee el valor analógico del LDR y si la luz es muy baja, enciende un LED.

import machine
import time

ldr_pin = machine.ADC(machine.Pin(36))  # ADC pin en ESP32
led_pin = machine.Pin(2, machine.Pin.OUT)  # LED indicador

umbral_luz = 1000  

while True:
    valor = ldr_pin.read()
    print("Valor LDR:", valor)
    if valor < umbral_luz:
        print("Luz muy baja, encendiendo LED")
        led_pin.value(1)
    else:
        led_pin.value(0)
    time.sleep(1)
