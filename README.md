# PROYECTO: Falcon ESP

## Descripción General
### https://youtube.com/@falconesp_32?si=bBbZBDxBUkOGDTZP @Video de Youtube


Este proyecto busca implementar un sistema completo de control y monitoreo para un dron basado en un microcontrolador con MicroPython (ESP32). El objetivo es integrar diferentes sensores, actuadores, y métodos de comunicación, incluyendo:

- Estabilización del dron mediante giroscopio/acelerómetro (MPU6050).
- Recepción y envío de datos por radio (NRF24L01).
- Control de cuatro motores (A, B, A1, B1) con PWM para variar su velocidad.
- Detección de obstáculos con sensor ultrasónico (HC-SR04).
- Monitoreo de luz ambiente con LDR y encendido de LED indicador.
- Obtención de ubicación GPS (NEO6M) para registro y envío a Firebase.
- Buzzer para señales auditivas (conectado/desconectado, advertencias).
- Conexión WiFi y envío de datos a Firebase.
- Diferentes modos de control (manual, automático, perfiles de vuelo).
- Ajuste de parámetros PID para estabilización avanzada.
- Mantenimiento de altitud y posición (modo estacionario).
- Monitoreo de estados y envío periódico de información.

## Principales Características

### Conexión a WiFi y Envío a Firebase
El código se conecta a una red WiFi configurada (SSID y PASS) y posteriormente utiliza `urequests` para enviar datos a un endpoint de Firebase. Esto permite registrar remotamente la posición, estado de motores y otros datos relevantes del dron.

### NRF24L01 - Comunicación RF
Se emplea el módulo NRF24L01 para recibir datos desde un emisor (transmisor). Estos datos pueden incluir:

- Estado de conexión (`CON:1` o `CON:0`)
- Ajuste de altitud (`ALT:xxx`)
- Movimientos (adelante, atrás, izquierda, derecha)
- Comandos avanzados (ajuste de PID, perfiles de vuelo, modo automático/estacionario)

Según el comando recibido, el receptor ajusta la velocidad de los motores o cambia su estado interno.

### Control de Motores con PWM
Se controlan cuatro motores (A, B, A1, B1) mediante PWM (0-255 → 0-1023 duty en ESP32).  
Los comandos recibidos pueden elevar todos a la misma velocidad, o mover el dron en alguna dirección aumentando o disminuyendo ciertos motores. Se aplican límites para evitar saturar los motores.

### Sensor Ultrasónico
El sensor ultrasónico (HC-SR04) mide la distancia frontal. Si hay un obstáculo muy cerca, el dron evita avanzar en esa dirección y emite un beep, previniendo colisiones.

### LDR y LED Indicador de Luz Baja
Un LDR conectado a un pin ADC mide la luz ambiente. Si el nivel es bajo, enciende un LED indicador. Además, el código avanzado puede cambiar el comportamiento dependiendo de la luminosidad (por ejemplo, incrementar altitud en condiciones de poca luz).

### GPS NEO6M
A través de UART, el dron lee datos del GPS. Una vez que el GPS obtiene fix (latitud y longitud válidas), envía estos datos a Firebase, registrando así la posición geográfica del dron.

### Buzzer
Un buzzer conectado a una salida digital emite pitidos cortos o largos en diferentes situaciones:

- Conexión establecida (beep corto).
- Conexión perdida (beep más largo).
- Advertencias, como obstáculos demasiado cerca.

### Estabilización con MPU6050
El MPU6050 proporciona valores de aceleración (y opcionalmente giroscopio). El proyecto implementa:

- Estabilización básica (sumando o restando un factor fijo según inclinación).
- Estabilización avanzada tipo PID, usando ajustes de kp, ki, kd para roll y pitch.
- Filtrado de valores, offsets para calibración, y modo estacionario para mantener el dron inmóvil.

Esta lógica corrige la inclinación del dron ajustando las velocidades de los motores para mantener ax ~0 y ay ~0.

### Altitud y Modos de Vuelo
Se incluye lógica (simulada) para mantener altitud estable usando PID, así como comandos para cambiar el setpoint de altitud.  
Modos disponibles:

- Modo estacionario: mantiene posición y altitud.
- Modo automático o manual.
- Perfiles de vuelo (NORMAL, SUAVE, AGRESIVO) que cambian parámetros PID.

### Registro y Telemetría
Se envían datos periódicamente a Firebase, se mantiene un log interno y se pueden agregar más endpoints para telemetría externa. Esto facilita el monitoreo remoto y el ajuste dinámico de parámetros.

## Estructura del Código
- Conexión WiFi y envío a Firebase.
- Definición de pines y funciones de utilidad (beep, medir_distancia, set_speed, etc.).
- Control de motores y funciones para moverse en cada dirección.
- Estabilización básica y avanzada (PID).
- Lectura GPS y envío a Firebase.
- Recepción de comandos por NRF24 y procesamiento (conexión, altitud, dirección, PID, modos de vuelo).
- Modo estacionario, reacción a luz, control de altitud, perfiles de vuelo.
- Bucle principal:
  - Lee LDR, ajusta LED.
  - Recibe datos NRF.
  - Verifica conexión.
  - Estabiliza (PID avanzado o simple).
  - Controla altitud, posición, evita colisiones.
  - Lee GPS periódicamente, envía datos.
  - Envía estado completo periódicamente a Firebase.
  - Aplica perfiles, mantiene integrales, mantenimiento del sistema.

## Conclusión
El proyecto Falcon ESP es un ejemplo extenso de cómo integrar múltiples componentes (sensores, actuadores, comunicación, estabilización) en un dron controlado por un ESP32 con MicroPython. Incluye desde tareas simples (encender un LED con un LDR) hasta control PID avanzado, comunicación inalámbrica NRF24, envío a Firebase vía WiFi, y lógicas complejas (modos de vuelo, perfiles PID).

Esta arquitectura permite adaptarse y escalar el proyecto según las necesidades, agregar más sensores, mejorar el control o integrar nuevas funcionalidades sin modificar la estructura base del proyecto. Además, brinda la flexibilidad de incluir muchos más detalles, permitiendo un desarrollo continuo y personalizado.

#### --------------------------------------------------------------------------------------------------------------
# PROYECTO: Falcon ESP - Módulo de Video en Tiempo Real con ESP32-CAM

## Descripción General

Este módulo forma parte del proyecto **Falcon ESP**, el cual integra múltiples componentes para el control y monitoreo de un dron inteligente. En esta sección se ha incorporado una **ESP32-CAM** a la infraestructura existente de Falcon ESP, permitiendo la transmisión de imágenes fijas (JPG) y video en tiempo real (stream MJPEG) a través de una conexión WiFi, ampliando así las capacidades del sistema.

La integración de la ESP32-CAM dentro del proyecto Falcon ESP permite que el dron:
- Ofrezca visión en primera persona (FPV) al operador.
- Genere registro visual del entorno en el que se mueve.
- Facilite la navegación, reconocimiento de obstáculos y "escenarios complejos".
- Envía información visual a los controladores en tierra o a sistemas externos.

## Características Principales

- **Integración Total en Falcon ESP:** El módulo de video funciona conjuntamente con el resto de componentes (MPU6050, NRF24, Ultrasonido, GPS, LDR, etc.).  
- **Conexión WiFi:** Usa la misma red WiFi configurada en Falcon ESP para envío de datos a Firebase y ahora también para la transmisión de video.  
- **Servidor HTTP Local:** La ESP32-CAM crea un servidor HTTP en el mismo dron, accesible desde un navegador en la misma red, permitiendo:
  - **`/jpg`:** Captura una imagen JPG al momento y la entrega al cliente.
  - **`/video`:** Ofrece un stream MJPEG, enviando imágenes de forma continua para simular video en tiempo real.
- **Configuración de Cámara Ajustable:** Se pueden modificar la resolución (framesize) y calidad JPEG según las necesidades del dron.  
- **Interacción con Otros Módulos:** El video puede emplearse para mejorar la lógica de estabilidad, detección de obstáculos, o asistencia a la navegación integrada en Falcon ESP.

## Requerimientos

- ESP32-CAM compatible con MicroPython.
- Ajustar SSID y contraseña WiFi, así como parámetros de cámara.
- Haber integrado previamente el entorno Falcon ESP con sus componentes de telemetría, sensores, y actuadores.


## Conclusión

La integración de la ESP32-CAM en el proyecto Falcon ESP amplía significativamente las capacidades del dron, no sólo dotándolo de una “visión” remota, sino también potenciando el uso de las imágenes para mejorar la toma de decisiones, el control y el monitoreo. Este es un paso importante hacia un dron más inteligente y versátil, capaz de interactuar con su entorno de manera más completa. Con esta funcionalidad, Falcon ESP se convierte en una plataforma más robusta, adaptable y escalable, abriendo la puerta a nuevas aplicaciones y desarrollos futuros en el ámbito de la robótica aérea.  
