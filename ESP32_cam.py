# PROYECTO: Stream de imágenes y video en tiempo real con ESP32-CAM para el monitoreo y revisión
#
# Este código crea un servidor HTTP en la ESP32-CAM para enviar imágenes
# en tiempo real (JPG individuales) y un stream de video MJPEG.
#
# Ajusta los parámetros de la cámara, inicia la cámara, lanza un servidor
# simple en el puerto 80 y provee dos endpoints:
# - /jpg: retorna una imagen JPG actual.
# - /video: retorna un stream MJPEG (video) a partir de imágenes sucesivas.
#
#
# se requiere el firmware específico para ESP32-CAM con soporte de cámara.
#

import machine
import time
import network
import socket
import gc
from machine import Pin
import esp32
import os

# -------- Configuración WiFi --------
WIFI_SSID = "ESPRED"
WIFI_PASS = "passwordesp"

def connect_wifi(ssid, password):
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    if not wlan.isconnected():
        print("Conectando a WiFi...")
        wlan.connect(ssid, password)
        while not wlan.isconnected():
            pass
    print("Conectado a WiFi:", wlan.ifconfig())

connect_wifi(WIFI_SSID, WIFI_PASS)
gc.collect()

# -------- Configuración Cámara --------

import esp32_camera

# Configuración de la cámara 
esp32_camera.init(framesize=esp32_camera.FRAME_SIZE_QVGA,  # QVGA 320x240
                  quality=10,                             # calidad JPG
                  fb_count=2)                             # framebuffers

# Nota: Ajustar framesize y quality según necesidades.

# -------- Funciones para capturar imágenes --------
def capturar_jpg():
    # Retorna una imagen JPG en bytes
    buf = esp32_camera.capture()
    return buf

# -------- Servidor HTTP --------
# Creamos un servidor TCP simple para responder a las peticiones
# /jpg y /video

def send_header(client, content_type="text/html", code=200, additional=None):
    client.write("HTTP/1.1 {} OK\r\n".format(code))
    client.write("Content-Type: {}\r\n".format(content_type))
    if additional:
        for a in additional:
            client.write(a+"\r\n")
    client.write("\r\n")

def handle_jpg(client):
    # Captura una imagen JPG y la envía
    img = capturar_jpg()
    send_header(client, content_type="image/jpeg")
    client.write(img)

def handle_video(client):
    # Stream MJPEG: enviar varias imágenes con multipart/x-mixed-replace
    boundary = "boundarydonotcross"
    headers = [
        "Cache-Control: no-cache",
        "Pragma: no-cache",
        "Content-Type: multipart/x-mixed-replace; boundary={}".format(boundary)
    ]
    client.write("HTTP/1.1 200 OK\r\n")
    for h in headers:
        client.write(h+"\r\n")
    client.write("\r\n")

    # Enviar frames en bucle
    try:
        while True:
            frame = capturar_jpg()
            client.write("--{}\r\n".format(boundary))
            client.write("Content-Type: image/jpeg\r\n")
            client.write("Content-Length: {}\r\n\r\n".format(len(frame)))
            client.write(frame)
            client.write("\r\n")
            time.sleep(0.05) # Controlar frame rate
    except:
        # Si el cliente cierra conexión, salimos
        pass

def handle_root(client):
    # Página simple con links a /jpg y /video
    html = """<html>
<head><title>ESP32-CAM</title></head>
<body>
<h1>ESP32-CAM Streaming</h1>
<p><a href="/jpg">Obtener JPG</a></p>
<p><a href="/video">Ver Video MJPEG</a></p>
</body></html>"""
    send_header(client)
    client.write(html)

def handle_404(client):
    send_header(client, code=404)
    client.write("404 Not Found")

def parse_request(req):
    # Obtener la primera línea
    lines = req.split('\r\n')
    if len(lines)>0:
        request_line = lines[0]
        parts = request_line.split(' ')
        if len(parts)>=2:
            method = parts[0]
            path = parts[1]
            return method, path
    return None, None

# Crear socket servidor
addr = socket.getaddrinfo('0.0.0.0', 80)[0][-1]
s = socket.socket()
s.bind(addr)
s.listen(5)
print("Servidor HTTP escuchando en puerto 80")

# Bucle principal del servidor
while True:
    try:
        client_sock, client_addr = s.accept()
        # print("Cliente conectado:", client_addr)
        client_sock.settimeout(2.0)
        req = client_sock.recv(1024)
        if req:
            req_str = req.decode('utf-8', 'ignore')
            method, path = parse_request(req_str)
            if path == "/":
                handle_root(client_sock)
            elif path == "/jpg":
                handle_jpg(client_sock)
            elif path == "/video":
                handle_video(client_sock)
            else:
                handle_404(client_sock)
        client_sock.close()
    except Exception as e:
        # Cualquier error se ignora en este ejemplo
        pass
    gc.collect()
