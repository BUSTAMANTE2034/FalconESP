# PROYECTO: Falcon ESP
# Código del Buzzer

import machine
import time

buzzer_pin = machine.Pin(14, machine.Pin.OUT)

def beep(t=0.5):
    buzzer_pin.value(1)
    time.sleep(t)
    buzzer_pin.value(0)

# Prueba: hacer sonar el buzzer 3 veces
for i in range(3):
    beep(0.2)
    time.sleep(0.2)
