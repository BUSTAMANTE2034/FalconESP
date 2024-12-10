# PROYECTO: Falcon ESP
# Código de Motores con PWM

import machine
import time

# Pines PWM para motores 
pwmA = machine.PWM(machine.Pin(4))
pwmB = machine.PWM(machine.Pin(5))
pwmA1 = machine.PWM(machine.Pin(18))
pwmB1 = machine.PWM(machine.Pin(19))

# Frecuencia del PWM
freq = 1000
pwmA.freq(freq)
pwmB.freq(freq)
pwmA1.freq(freq)
pwmB1.freq(freq)

# Función para ajustar velocidad
def set_speed(motor_pwm, speed):
    # speed: 0-255 se mapea a duty 0-1023 (en ESP32)
    duty = int((speed/255)*1023)
    motor_pwm.duty(duty)

# Inicializamos en 0
set_speed(pwmA, 0)
set_speed(pwmB, 0)
set_speed(pwmA1, 0)
set_speed(pwmB1, 0)

# Funciones de movimiento (conceptual)
def elevar_todos(vel):
    set_speed(pwmA, vel)
    set_speed(pwmB, vel)
    set_speed(pwmA1, vel)
    set_speed(pwmB1, vel)
    print("Elevando todos los motores a velocidad:", vel)
    
def mover_derecha(vel):
    # Aumentar A1 y B1
    set_speed(pwmA1, vel)
    set_speed(pwmB1, vel)
    print("Moviendo a la derecha con velocidad:", vel)
    
def mover_izquierda(vel):
    # Aumentar A y B
    set_speed(pwmA, vel)
    set_speed(pwmB, vel)
    print("Moviendo a la izquierda con velocidad:", vel)

def mover_adelante(vel):
    # Aumentar A y A1
    set_speed(pwmA, vel)
    set_speed(pwmA1, vel)
    print("Moviendo adelante con velocidad:", vel)

def mover_atras(vel):
    # Aumentar B y B1
    set_speed(pwmB, vel)
    set_speed(pwmB1, vel)
    print("Moviendo atras con velocidad:", vel)

# Prueba: elevar a 100, esperar, mover derecha a 150, etc.
elevar_todos(100)
time.sleep(2)
mover_derecha(150)
time.sleep(2)
mover_izquierda(200)
time.sleep(2)
mover_adelante(255)
time.sleep(2)
mover_atras(50)
time.sleep(2)

# Apagar
elevar_todos(0)
