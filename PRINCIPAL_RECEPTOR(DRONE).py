# PROYECTO: Falcon ESP
# Código principal del Receptor FalconESP
# Este código integra:
# - Estabilización con MPU6050 (giroscopio)
# - Recepción de datos por NRF24 (conexión, velocidad, movimientos)
# - Control de motores con PWM
# - Sensor ultrasonico para evitar colisiones
# - LDR y LED indicador de luz baja
# - GPS para obtener ubicación y enviar a Firebase
# - Buzzer para indicar estados de conexión
# - Envío de datos a Firebase por WiFi
#
# Se agregan funciones, variables, y lógica para estabilización avanzada.

import machine
import time
from mpu6050 import MPU6050
from nrf24l01 import NRF24L01
from micropyGPS import MicropyGPS
import network
import urequests

WIFI_SSID = "ESPRED"
WIFI_PASS = "passwordesp"
FIREBASE_URL = "https://flaconesp.firebaseio.com/data.json"

def connect_wifi(ssid, password):
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    if not wlan.isconnected():
        print("Conectando a WiFi...")
        wlan.connect(ssid, password)
        while not wlan.isconnected():
            pass
    print("Conectado WiFi:", wlan.ifconfig())

def send_to_firebase(data):
    try:
        res = urequests.post(FIREBASE_URL, json=data)
        print("Enviado a Firebase:", res.text)
        res.close()
    except Exception as e:
        print("Error Firebase:", e)

connect_wifi(WIFI_SSID, WIFI_PASS)

# Pines y periféricos
led_conexion = machine.Pin(2, machine.Pin.OUT)
led_conexion.value(0)
buzzer_pin = machine.Pin(14, machine.Pin.OUT)
def beep(t=0.1):
    buzzer_pin.value(1)
    time.sleep(t)
    buzzer_pin.value(0)
    
ldr_pin = machine.ADC(machine.Pin(36))
umbral_luz = 1000
led_luz = machine.Pin(4, machine.Pin.OUT)
led_luz.value(0)

trig = machine.Pin(13, machine.Pin.OUT)
echo = machine.Pin(12, machine.Pin.IN)
def medir_distancia():
    trig.value(0)
    time.sleep_us(5)
    trig.value(1)
    time.sleep_us(10)
    trig.value(0)
    while echo.value() == 0:
        start = time.ticks_us()
    while echo.value() == 1:
        end = time.ticks_us()
    duracion = end - start
    distancia = (duracion / 2) / 29.1
    return distancia

pwmA = machine.PWM(machine.Pin(27))
pwmB = machine.PWM(machine.Pin(26))
pwmA1 = machine.PWM(machine.Pin(25))
pwmB1 = machine.PWM(machine.Pin(33))
freq = 1000
pwmA.freq(freq)
pwmB.freq(freq)
pwmA1.freq(freq)
pwmB1.freq(freq)

def set_speed(motor_pwm, speed):
    if speed < 0: speed = 0
    if speed > 255: speed = 255
    duty = int((speed/255)*1023)
    motor_pwm.duty(duty)

velA = 0
velB = 0
velA1 = 0
velB1 = 0

def aplicar_velocidades():
    set_speed(pwmA, velA)
    set_speed(pwmB, velB)
    set_speed(pwmA1, velA1)
    set_speed(pwmB1, velB1)

def elevar_todos(vel):
    global velA, velB, velA1, velB1
    velA = vel
    velB = vel
    velA1 = vel
    velB1 = vel
    aplicar_velocidades()

def mover_derecha(delta):
    global velA1, velB1
    dist = medir_distancia()
    if dist < 20:
        print("Objeto a la derecha, no avanzar!")
        beep(0.3)
    else:
        velA1 += delta
        velB1 += delta
        aplicar_velocidades()

def mover_izquierda(delta):
    global velA, velB
    dist = medir_distancia()
    if dist < 20:
        print("Objeto a la izquierda, no avanzar!")
        beep(0.3)
    else:
        velA += delta
        velB += delta
        aplicar_velocidades()

def mover_adelante(delta):
    global velA, velA1
    dist = medir_distancia()
    if dist < 20:
        print("Objeto adelante, no avanzar!")
        beep(0.3)
    else:
        velA += delta
        velA1 += delta
        aplicar_velocidades()

def mover_atras(delta):
    global velB, velB1
    dist = medir_distancia()
    if dist < 20:
        print("Objeto atras, no avanzar!")
        beep(0.3)
    else:
        velB += delta
        velB1 += delta
        aplicar_velocidades()

i2c = machine.I2C(scl=machine.Pin(22), sda=machine.Pin(21))
mpu = MPU6050(i2c)

# Estabilización 
def estabilizar():
    ax, ay, az = mpu.get_accel_data()
    factor = 10
    global velA, velB, velA1, velB1
    if ax > 0.5:
        velA -= factor
        velB -= factor
        velA1 += factor
        velB1 += factor
    elif ax < -0.5:
        velA += factor
        velB += factor
        velA1 -= factor
        velB1 -= factor
    if ay > 0.5:
        velA -= factor
        velA1 -= factor
        velB += factor
        velB1 += factor
    elif ay < -0.5:
        velA += factor
        velA1 += factor
        velB -= factor
        velB1 -= factor
    aplicar_velocidades()

uart_gps = machine.UART(2, rx=16, tx=17, baudrate=9600)
gps = MicropyGPS()

def leer_gps_y_enviar():
    if uart_gps.any():
        data = uart_gps.read()
        for b in data:
            gps.update(chr(b))
        if gps.fix_stat > 0:
            lat = gps.latitude[0] + gps.latitude[1]/60
            lon = gps.longitude[0] + gps.longitude[1]/60
            if gps.latitude_card == 'S':
                lat = -lat
            if gps.longitude_card == 'W':
                lon = -lon
            send_to_firebase({"lat": lat, "lon": lon})
            print("GPS lat:", lat, "lon:", lon)

csn = machine.Pin(15, machine.Pin.OUT)
ce = machine.Pin(16, machine.Pin.OUT)
spi = machine.SPI(1, baudrate=4000000, polarity=0, phase=0)
nrf = NRF24L01(spi, ce, csn, channel=60, payload_size=32)
nrf.open_rx_pipe(1, b"1Node")
nrf.listen(True)

conexion_activa = False

def procesar_datos_nrf(dato):
    global conexion_activa
    mensaje = dato.decode()
    if mensaje.startswith("CON:"):
        estado_con = mensaje.split(":")[1]
        if estado_con == "1":
            print("Conexión establecida")
            conexion_activa = True
            led_conexion.value(1)
            beep(0.2)
        else:
            print("Conexión perdida")
            conexion_activa = False
            led_conexion.value(0)
            beep(0.5)
    elif mensaje.startswith("ALT:"):
        alt_str = mensaje.split(":")[1]
        alt_val = int(alt_str)
        elevar_todos(alt_val)
    elif mensaje.startswith("F:"):
        val = int(mensaje.split(":")[1])
        if val == 1:
            mover_adelante(10)
    elif mensaje.startswith("B:"):
        val = int(mensaje.split(":")[1])
        if val == 1:
            mover_atras(10)
    elif mensaje.startswith("L:"):
        val = int(mensaje.split(":")[1])
        if val == 1:
            mover_izquierda(10)
    elif mensaje.startswith("R:"):
        val = int(mensaje.split(":")[1])
        if val == 1:
            mover_derecha(10)


#factores ajustables y registros históricos para filtros.
pid_kp_roll = 10
pid_ki_roll = 0.1
pid_kd_roll = 5

pid_kp_pitch = 10
pid_ki_pitch = 0.1
pid_kd_pitch = 5

roll_error_prev = 0.0
pitch_error_prev = 0.0
roll_integral = 0.0
pitch_integral = 0.0

# Offset del MPU para calibración
mpu_ax_offset = 0.0
mpu_ay_offset = 0.0
mpu_az_offset = 0.0

# Registro histórico para filtrado
roll_history = []
pitch_history = []
history_len = 20

# Función para calibrar offsets MPU
def calibrar_mpu(n=100):
    global mpu_ax_offset, mpu_ay_offset, mpu_az_offset
    sum_ax=0;sum_ay=0;sum_az=0
    for i in range(n):
        ax,ay,az=mpu.get_accel_data()
        sum_ax+=ax;sum_ay+=ay;sum_az+=az
        time.sleep(0.01)
    mpu_ax_offset=(sum_ax/n)
    mpu_ay_offset=(sum_ay/n)
    mpu_az_offset=(sum_az/n)
    print("Offsets MPU:",mpu_ax_offset,mpu_ay_offset,mpu_az_offset)

# Llamamos una vez al inicio
calibrar_mpu()

# Función para agregar datos a histórico
def add_history(h, val):
    h.append(val)
    if len(h)>history_len:
        h.pop(0)

# Filtro simple (media)
def filtrar(h):
    if len(h)==0:return 0
    return sum(h)/len(h)

# función de estabilización avanzada usando PID
def estabilizar_avanzado():
    global velA, velB, velA1, velB1
    global roll_error_prev, pitch_error_prev, roll_integral, pitch_integral

    ax,ay,az=mpu.get_accel_data()
    # Ajuste por offset
    ax -= mpu_ax_offset
    ay -= mpu_ay_offset
    # Agregar a histórico
    add_history(roll_history, ax)
    add_history(pitch_history, ay)

    # Filtrado
    ax_f = filtrar(roll_history)
    ay_f = filtrar(pitch_history)

    # Queremos ax_f ~ 0, ay_f ~ 0
    roll_error = ax_f
    pitch_error = ay_f

    # PID Roll
    roll_integral += roll_error
    roll_deriv = roll_error - roll_error_prev
    roll_error_prev = roll_error
    roll_control = (pid_kp_roll*roll_error)+(pid_ki_roll*roll_integral)+(pid_kd_roll*roll_deriv)

    # PID Pitch
    pitch_integral += pitch_error
    pitch_deriv = pitch_error - pitch_error_prev
    pitch_error_prev = pitch_error
    pitch_control = (pid_kp_pitch*pitch_error)+(pid_ki_pitch*pitch_integral)+(pid_kd_pitch*pitch_deriv)

    # Ajuste motores
    # Roll: roll_control >0 => inclinado derecha => compensar como antes
    velA -= roll_control
    velB -= roll_control
    velA1+= roll_control
    velB1+= roll_control

    # Pitch
    velA -= pitch_control
    velA1-= pitch_control
    velB += pitch_control
    velB1+= pitch_control

    # Limites
    if velA<0:velA=0
    if velA>255:velA=255
    if velB<0:velB=0
    if velB>255:velB=255
    if velA1<0:velA1=0
    if velA1>255:velA1=255
    if velB1<0:velB1=0
    if velB1>255:velB1=255

    aplicar_velocidades()

# Añadimos control para mantener altitud fija basada en la velocidad ALT recibida
altitud_setpoint = 0
altitud_actual = 0
altitud_error_prev = 0
altitud_integral = 0
pid_kp_alt = 5
pid_ki_alt = 0.05
pid_kd_alt = 2


# La altitud_actual se deduce de velA,B,A1,B1 promedio
def estimar_altitud():
    # Valor: promedio de velocidades simula sustentación
    prom = (velA+velB+velA1+velB1)/4
    # altitud ~ prom/2
    return prom/2

def controlar_altitud():
    global altitud_setpoint, altitud_actual, altitud_error_prev, altitud_integral
    altitud_actual = estimar_altitud()
    error_alt = altitud_setpoint - altitud_actual
    altitud_integral += error_alt
    alt_deriv = error_alt - altitud_error_prev
    altitud_error_prev = error_alt
    control_alt = (pid_kp_alt*error_alt)+(pid_ki_alt*altitud_integral)+(pid_kd_alt*alt_deriv)
    # Ajustar todos los motores uniformemente
    global velA,velB,velA1,velB1
    velA += control_alt
    velB += control_alt
    velA1+= control_alt
    velB1+= control_alt
    if velA<0:velA=0
    if velA>255:velA=255
    if velB<0:velB=0
    if velB>255:velB=255
    if velA1<0:velA1=0
    if velA1>255:velA1=255
    if velB1<0:velB1=0
    if velB1>255:velB1=255
    aplicar_velocidades()

# Log de datos para depuración
log_data = []
def log_estado():
    # Registra estado básico: ax_f,ay_f,altitud_actual,velA...
    # Solo guardamos las líneas
    if len(log_data)>50:
        log_data.pop(0)
    ax,ay,az=mpu.get_accel_data()
    ax_off = ax - mpu_ax_offset
    ay_off = ay - mpu_ay_offset
    log_data.append((time.ticks_ms(),ax_off,ay_off,velA,velB,velA1,velB1,altitud_actual))

# Ajuste dinámico de PID (por si se reciben comandos para cambiar PID)
def ajustar_pid_roll(p, i, d):
    global pid_kp_roll, pid_ki_roll, pid_kd_roll
    pid_kp_roll=p
    pid_ki_roll=i
    pid_kd_roll=d

def ajustar_pid_pitch(p, i, d):
    global pid_kp_pitch, pid_ki_pitch, pid_kd_pitch
    pid_kp_pitch=p
    pid_ki_pitch=i
    pid_kd_pitch=d

def ajustar_pid_alt(p, i, d):
    global pid_kp_alt, pid_ki_alt, pid_kd_alt
    pid_kp_alt=p
    pid_ki_alt=i
    pid_kd_alt=d

# Podríamos recibir nuevos comandos NRF para cambiar PID
def procesar_comando_avanzado(mensaje):
    # Ej: "PIDR:kp,ki,kd" para roll, "PIDP:..." pitch, "PIDA:..." alt
    # Ej: "SETA:100" para set altitud a 100
    if mensaje.startswith("PIDR:"):
        vals=mensaje.split(":")[1].split(",")
        p=float(vals[0]);i=float(vals[1]);d=float(vals[2])
        ajustar_pid_roll(p,i,d)
    elif mensaje.startswith("PIDP:"):
        vals=mensaje.split(":")[1].split(",")
        p=float(vals[0]);i=float(vals[1]);d=float(vals[2])
        ajustar_pid_pitch(p,i,d)
    elif mensaje.startswith("PIDA:"):
        vals=mensaje.split(":")[1].split(",")
        p=float(vals[0]);i=float(vals[1]);d=float(vals[2])
        ajustar_pid_alt(p,i,d)
    elif mensaje.startswith("SETA:"):
        global altitud_setpoint
        alt_str=mensaje.split(":")[1]
        alt_val=int(alt_str)
        altitud_setpoint=alt_val

# Sobrescribir procesar_datos_nrf para incluir estos comandos
old_procesar_datos_nrf = procesar_datos_nrf
def procesar_datos_nrf(dato):
    mensaje=dato.decode()
    # Primero verificar si es un comando avanzado
    if mensaje.startswith("PIDR:") or mensaje.startswith("PIDP:") or mensaje.startswith("PIDA:") or mensaje.startswith("SETA:"):
        procesar_comando_avanzado(mensaje)
    else:
        old_procesar_datos_nrf(dato)

# Lógica de seguridad: Si no hay conexión por mucho tiempo, disminuir potencia
last_conexion_time = time.ticks_ms()
def verificar_conexion():
    global conexion_activa, last_conexion_time, velA,velB,velA1,velB1
    if conexion_activa:
        if time.ticks_diff(time.ticks_ms(), last_conexion_time) > 10000:
            # 2s sin datos nuevos, asumir desconexión
            conexion_activa=False
            led_conexion.value(0)
            beep(0.5)
            # Disminuir potencia gradualmente
            velA=velB=velA1=velB1=0
            aplicar_velocidades()

# Revisar datos NRF
def recibir_nrf():
    global last_conexion_time
    if nrf.any():
        dato=nrf.recv()
        mensaje=dato.decode()
        if mensaje.startswith("CON:1"):
            last_conexion_time=time.ticks_ms()
        procesar_datos_nrf(dato)

# Control de distancia frontal también a la estabilización
# Si muy cerca, no inclinar adelante.
def limitaciones_por_distancia():
    dist=medir_distancia()
    if dist<20:
        # Si dron inclinado hacia adelante, intentar revertir
        # Detectar si ay>0 (inclinando adelante)
        ax,ay,az=mpu.get_accel_data()
        ay-=mpu_ay_offset
        if ay>0.5:
            # Reducir movimiento adelante
            global velA,velA1
            velA=velA1=min(velA,velA1)*0.5
            aplicar_velocidades()

# Control de luz: si luz baja, aumentar altitud_setpoint (ejemplo ficticio)
def reaccion_luz():
    val_ldr=ldr_pin.read()
    if val_ldr<umbral_luz:
        # Ajustar altitud_setpoint un poco
        global altitud_setpoint
        altitud_setpoint+=1
        if altitud_setpoint>200:
            altitud_setpoint=200
    else:
        # Si hay buena luz, reducir altitud_setpoint
        global altitud_setpoint
        altitud_setpoint-=1
        if altitud_setpoint<0:
            altitud_setpoint=0

# Funciones para modo estacionario (mantener altitud y posición)
modo_estacionario=False
def activar_modo_estacionario():
    global modo_estacionario
    modo_estacionario=True

def desactivar_modo_estacionario():
    global modo_estacionario
    modo_estacionario=False

def mantener_estacionario():
    # Si modo estacionario, objetivo ax=0,ay=0 y altitud fija
    # Ya estabilizar_avanzado controla roll/pitch
    # Solo controlar altitud
    controlar_altitud()

# Lectura periódica GPS con intervalos
gps_counter=0
gps_interval=20

# Registro periódico a Firebase de estado completo
estado_counter=0
estado_interval=50

def enviar_estado_completo():
    data={
        "conexion":conexion_activa,
        "velA":velA,"velB":velB,"velA1":velA1,"velB1":velB1,
        "ax_off":(mpu.get_accel_data()[0]-mpu_ax_offset),
        "ay_off":(mpu.get_accel_data()[1]-mpu_ay_offset),
        "altitud_setpoint":altitud_setpoint,
        "altitud_actual":altitud_actual
    }
    send_to_firebase(data)

# Más variables
max_speed=255
min_speed=0
def asegurar_rangos():
    global velA,velB,velA1,velB1
    if velA<min_speed:velA=min_speed
    if velA>max_speed:velA=max_speed
    if velB<min_speed:velB=min_speed
    if velB>max_speed:velB=max_speed
    if velA1<min_speed:velA1=min_speed
    if velA1>max_speed:velA1=max_speed
    if velB1<min_speed:velB1=min_speed
    if velB1>max_speed:velB1=max_speed

def control_motores_con_aseguramiento():
    asegurar_rangos()
    aplicar_velocidades()

# Lógica de modo manual vs modo automático
modo_automatico=True
def set_modo_automatico(auto):
    global modo_automatico
    modo_automatico=auto

# Si modo_automatico=False, no usar estabilizar_avanzado, solo estabilizar simple
# Si modo_automatico=True, usar estabilizar_avanzado
# Opción a recibir comando "AUTO:1" o "AUTO:0"
def procesar_comando_extra(mensaje):
    if mensaje.startswith("AUTO:"):
        val=mensaje.split(":")[1]
        if val=="1":
            set_modo_automatico(True)
        else:
            set_modo_automatico(False)

# Integrar esto en procesar_datos_nrf
old_pd_nrf=procesar_datos_nrf
def procesar_datos_nrf(dato):
    mensaje=dato.decode()
    if mensaje.startswith("AUTO:"):
        procesar_comando_extra(mensaje)
    else:
        old_pd_nrf(dato)

# Tiempo de ciclo y compensar PID según frecuencia
last_time=time.ticks_ms()
def ciclo_tiempo():
    global last_time
    current=time.ticks_ms()
    dt=time.ticks_diff(current,last_time)/1000.0
    last_time=current
    return dt


def fallback_estabilizar():
    # En caso de fallo, usar estabilización simple
    estabilizar()

def uso_avanzado_estabilizar():
    # Decide si usar estabilizar avanzado o simple
    if modo_automatico:
        estabilizar_avanzado()
    else:
        fallback_estabilizar()

# Arrays para logs
roll_log=[]
pitch_log=[]
alt_log=[]

def log_masivo():
    # Guardar datos en logs masivos
    if len(roll_log)>200:
        roll_log.pop(0)
    if len(pitch_log)>200:
        pitch_log.pop(0)
    if len(alt_log)>200:
        alt_log.pop(0)
    ax,ay,az=mpu.get_accel_data()
    roll_log.append(ax-mpu_ax_offset)
    pitch_log.append(ay-mpu_ay_offset)
    alt_log.append(altitud_actual)

# Control de tiempo en estabilizar avanzado
def estabilizar_avanzado_con_dt():
    # Igual que estabilizar_avanzado pero uso dt si quisiéramos
    # Sólo lo llamamos
    estabilizar_avanzado()


# Podemos simular distintos perfiles de vuelo
perfil_vuelo="NORMAL"
def cambiar_perfil(p):
    global perfil_vuelo
    perfil_vuelo=p

def aplicar_perfil():
    # Según perfil, ajustar PID
    if perfil_vuelo=="SUAVE":
        ajustar_pid_roll(5,0.05,2)
        ajustar_pid_pitch(5,0.05,2)
        ajustar_pid_alt(2,0.02,1)
    elif perfil_vuelo=="AGRESIVO":
        ajustar_pid_roll(20,0.2,10)
        ajustar_pid_pitch(20,0.2,10)
        ajustar_pid_alt(10,0.1,5)
    else:
        ajustar_pid_roll(10,0.1,5)
        ajustar_pid_pitch(10,0.1,5)
        ajustar_pid_alt(5,0.05,2)

# Comando para cambiar perfil
def procesar_comando_perfil(mensaje):
    if mensaje.startswith("PERFIL:"):
        p=mensaje.split(":")[1]
        cambiar_perfil(p)
        aplicar_perfil()

old_pd_nrf2=procesar_datos_nrf
def procesar_datos_nrf(dato):
    mensaje=dato.decode()
    if mensaje.startswith("PERFIL:"):
        procesar_comando_perfil(mensaje)
    else:
        old_pd_nrf2(dato)


# Función de mantenimiento
def mantenimiento_sistema():
    # Ajustar integral si se hace muy grande
    global roll_integral,pitch_integral,altitud_integral
    if abs(roll_integral)>1000:
        roll_integral=roll_integral*0.9
    if abs(pitch_integral)>1000:
        pitch_integral=pitch_integral*0.9
    if abs(altitud_integral)>1000:
        altitud_integral=altitud_integral*0.9

#Límites de velocidad
def ajustar_limites_speed(min_s, max_s):
    global min_speed, max_speed
    min_speed=min_s
    max_speed=max_s

def resetear_integrales():
    global roll_integral,pitch_integral,altitud_integral
    roll_integral=0
    pitch_integral=0
    altitud_integral=0

def centrado_emergencia():
    # Si algo sale mal, centrar motores
    global velA,velB,velA1,velB1
    velA=velB=velA1=velB1=100
    aplicar_velocidades()


# Bucle principal
contador=0
while True:
    val_ldr = ldr_pin.read()
    if val_ldr < umbral_luz:
        led_luz.value(1)
    else:
        led_luz.value(0)
    
    recibir_nrf()
    verificar_conexion()
    leer_sensores_extra()

    # Estabilizar dependiendo del modo
    uso_avanzado_estabilizar()

    # Control altitud si modo estacionario
    if modo_estacionario:
        mantener_estacionario()

    limitaciones_por_distancia()
    reaccion_luz()

    gps_counter+=1
    if gps_counter>=gps_interval:
        leer_gps_y_enviar()
        gps_counter=0

    estado_counter+=1
    if estado_counter>=estado_interval:
        enviar_estado_completo()
        estado_counter=0

    log_masivo()
    mantenimiento_sistema()
    ajustar_velocidades_direccionales()
    check_bateria()
    enviar_telemetria()
    leer_sensor_redundante()
    leer_parametros_externos()

    contador+=1
    if contador%100==0:
        cambiar_perfil("SUAVE")
        aplicar_perfil()
    if contador%200==0:
        cambiar_perfil("AGRESIVO")
        aplicar_perfil()
    if contador%300==0:
        cambiar_perfil("NORMAL")
        aplicar_perfil()

    time.sleep(0.2)
