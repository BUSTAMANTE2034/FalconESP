# PROYECTO: Falcon ESP
# Código del GPS NEO6M

import machine
import time
from micropyGPS import MicropyGPS

uart = machine.UART(2, rx=16, tx=17, baudrate=9600)  # Ajustar pines según hardware
gps = MicropyGPS()

print("Iniciando GPS...")

while True:
    # Leer datos NMEA desde UART
    if uart.any():
        data = uart.read()
        for b in data:
            stat = gps.update(chr(b))
        # Si tenemos fix, mostrar lat/long
        if gps.fix_stat > 0:
            print("GPS Conectado")
            lat = gps.latitude[0] + gps.latitude[1]/60
            lon = gps.longitude[0] + gps.longitude[1]/60
            # Ajuste de signo según gps.latitude_card y gps.longitude_card
            if gps.latitude_card == 'S':
                lat = -lat
            if gps.longitude_card == 'W':
                lon = -lon
            print("Latitud: ", lat, "Longitud:", lon)
        else:
            print("Buscando satélites...")
    time.sleep(1)
