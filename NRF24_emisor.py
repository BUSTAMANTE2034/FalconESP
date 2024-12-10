# PROYECTO: Falcon ESP
# Código de Emisor NRF24

from machine import Pin, SPI
from nrf24l01 import NRF24L01
import time

# Pines para NRF24 
csn = Pin(15, Pin.OUT)
ce = Pin(16, Pin.OUT)
spi = SPI(1, baudrate=4000000, polarity=0, phase=0)

nrf = NRF24L01(spi, ce, csn, channel=60, payload_size=32)
nrf.open_tx_pipe(b"1Node")
nrf.open_rx_pipe(1, b"2Node")

print("Emisor NRF24 iniciado, enviando datos...")

contador = 0
while True:
    mensaje = "Hola{}".format(contador)
    enviado = nrf.send(mensaje.encode())
    if enviado:
        print("Enviado:", mensaje)
    else:
        print("Error al enviar")
    contador += 1
    time.sleep(1)
