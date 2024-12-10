# PROYECTO: Falcon ESP
# Código principal del Emisor FalconESP
# Este código integra:
# - Joystick para controlar dirección
# - Envío de datos por NRF24 (conexión, velocidad, movimientos)
# - LED y buzzer para estados
# - Envío de datos a Firebase vía WiFi

import machine
import time
from nrf24l01 import NRF24L01
import network
import urequests

WIFI_SSID = "ESPRED"
WIFI_PASS = "passwordesp"
FIREBASE_URL = "https://falconesp.firebaseio.com/control.json"

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

# LED y Buzzer
led_conexion = machine.Pin(2, machine.Pin.OUT)
buzzer_pin = machine.Pin(14, machine.Pin.OUT)

def beep(t=0.1):
    buzzer_pin.value(1)
    time.sleep(t)
    buzzer_pin.value(0)

# Asumimos que la conexión al receptor está establecida desde el inicio
conexion_establecida = True
led_conexion.value(1)
beep(0.2)

# Joystick
adc_x = machine.ADC(machine.Pin(34))
adc_y = machine.ADC(machine.Pin(35))
centro = 2048
umbral = 300

# Potenciómetro para altura (o velocidad)
adc_alt = machine.ADC(machine.Pin(32))

# NRF24 Emisor
csn = machine.Pin(15, machine.Pin.OUT)
ce = machine.Pin(16, machine.Pin.OUT)
spi = machine.SPI(1, baudrate=4000000, polarity=0, phase=0)
nrf = NRF24L01(spi, ce, csn, channel=60, payload_size=32)
nrf.open_tx_pipe(b"1Node")
nrf.open_rx_pipe(1, b"2Node")

def enviar_nrf(mensaje):
    intentos = 3
    for i in range(intentos):
        if nrf.send(mensaje.encode()):
            print("Enviado:", mensaje)
            return
        else:
            print("Reintentando enviar:", mensaje)
            time.sleep(0.1)
    print("No se pudo enviar:", mensaje)

# Función para enviar estado de conexión
def enviar_conexion(estado):
    # estado True/False
    val = "1" if estado else "0"
    enviar_nrf("CON:" + val)

# Función para enviar altura
def enviar_altura(alt):
    enviar_nrf("ALT:" + str(alt))

# Funciones para enviar dirección
def enviar_adelante():
    enviar_nrf("F:1")
def enviar_atras():
    enviar_nrf("B:1")
def enviar_izquierda():
    enviar_nrf("L:1")
def enviar_derecha():
    enviar_nrf("R:1")

# Bucle principal
while True:
    # Leer joystick
    x_val = adc_x.read()
    y_val = adc_y.read()
    dx = x_val - centro
    dy = y_val - centro
    
    # Leer altura
    alt_val_raw = adc_alt.read()  # 0-4095
    alt_val = int((alt_val_raw/4095)*255)

    # Enviar altura periódicamente
    enviar_altura(alt_val)

    # Detectar dirección
    if dx > umbral:
        enviar_derecha()
    elif dx < -umbral:
        enviar_izquierda()
        
    if dy > umbral:
        enviar_adelante()
    elif dy < -umbral:
        enviar_atras()

    # Enviar datos a Firebase
    # Enviar estado del control (direcciones y altura)
    data = {
        "conexion": conexion_establecida,
        "altitud": alt_val,
        "direccion_x": dx,
        "direccion_y": dy
    }
    send_to_firebase(data)

    time.sleep(0.5)
