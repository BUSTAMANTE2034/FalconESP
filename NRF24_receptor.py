# PROYECTO: Falcon ESP
# Código de Receptor NRF24

from machine import Pin, SPI
from nrf24l01 import NRF24L01
import time

# Pines para NRF24 
csn = Pin(15, Pin.OUT)
ce = Pin(16, Pin.OUT)
spi = SPI(1, baudrate=4000000, polarity=0, phase=0)

nrf = NRF24L01(spi, ce, csn, channel=60, payload_size=32)
nrf.open_rx_pipe(1, b"1Node")
nrf.listen(True)

print("Receptor NRF24 iniciado, esperando datos...")

while True:
    if nrf.any():
        dato = nrf.recv()
        print("Recibido:", dato)
    time.sleep(0.1)
