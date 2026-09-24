from picamera2 import Picamera2
import cv2
import numpy as np
import time
import RPi.GPIO as GPIO
import serial
import threading
import math
import os
import smbus2

os.environ["DISPLAY"] = ":0"

# ============================================================
# GPIO
# ============================================================

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

BOTON = 16
GPIO.setup(BOTON, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# SERVO 180°
SERVO_PIN = 18
GPIO.setup(SERVO_PIN, GPIO.OUT)

# LED VERDE - ROBOT LISTO
LED_VERDE = 21
GPIO.setup(LED_VERDE, GPIO.OUT)
GPIO.output(LED_VERDE, GPIO.LOW)

# MOTOR
MOTOR_IN3 = 17
MOTOR_IN4 = 27
MOTOR_EN = 22

GPIO.setup(MOTOR_IN3, GPIO.OUT)
GPIO.setup(MOTOR_IN4, GPIO.OUT)
GPIO.setup(MOTOR_EN, GPIO.OUT)


# ============================================================
# SERVO 180°
# ============================================================

SERVO_CENTRO = 84
SERVO_IZQUIERDA = 65
SERVO_DERECHA = 115

SERVO_MIN = 60
SERVO_MAX = 120

INVERTIR_SERVO = False

servo_angulo_actual = SERVO_CENTRO
servo_objetivo = SERVO_CENTRO

servo_pwm = GPIO.PWM(SERVO_PIN, 50)
servo_pwm.start(0)

servo_activo = False
ultimo_movimiento_servo = 0.0


def activar_servo():
    global servo_activo
    global ultimo_movimiento_servo

    if not servo_activo:
        servo_pwm.ChangeDutyCycle(2.5 + (SERVO_CENTRO / 180.0) * 10.0)
        servo_activo = True
        ultimo_movimiento_servo = time.monotonic()
        time.sleep(0.15)


def desactivar_servo():
    global servo_activo

    if servo_activo:
        servo_pwm.ChangeDutyCycle(0)
        servo_activo = False


# ------------------------------------------------------------
# CONTROL DE SERVO
# ------------------------------------------------------------

SERVO_PASO_RAPIDO = 4.0
SERVO_PASO_MEDIO = 2.5
SERVO_PASO_FIN = 1.2
SERVO_INTERVALO = 0.015


def establecer_objetivo_servo(angulo):
    global servo_objetivo
    servo_objetivo = max(SERVO_MIN, min(SERVO_MAX, float(angulo)))


def actualizar_servo():
    global servo_angulo_actual
    global ultimo_movimiento_servo

    if not servo_activo:
        return

    ahora = time.monotonic()

    if ahora - ultimo_movimiento_servo < SERVO_INTERVALO:
        return

    diferencia = servo_objetivo - servo_angulo_actual
    distancia = abs(diferencia)

    if distancia < 0.6:
        servo_angulo_actual = servo_objetivo
    else:
        if distancia > 18:
            paso = SERVO_PASO_RAPIDO
        elif distancia > 7:
            paso = SERVO_PASO_MEDIO
        else:
            paso = SERVO_PASO_FIN

        if diferencia > 0:
            servo_angulo_actual += min(paso, diferencia)
        else:
            servo_angulo_actual -= min(paso, -diferencia)

    servo_angulo_actual = max(SERVO_MIN, min(SERVO_MAX, servo_angulo_actual))
    salida = servo_angulo_actual

    if INVERTIR_SERVO:
        salida = 180 - salida

    duty = 2.5 + (salida / 180.0) * 10.0
    servo_pwm.ChangeDutyCycle(duty)
    ultimo_movimiento_servo = ahora


def mover_servo_suave(angulo):
    establecer_objetivo_servo(angulo)


# ============================================================
# MOTOR
# ============================================================

motor_pwm = GPIO.PWM(MOTOR_EN, 1000)
motor_pwm.start(0)

VELOCIDAD = 89


def motor_avanzar(velocidad=VELOCIDAD):
    GPIO.output(MOTOR_IN3, GPIO.HIGH)
    GPIO.output(MOTOR_IN4, GPIO.LOW)
    motor_pwm.ChangeDutyCycle(velocidad)


def motor_retroceder(velocidad=90):
    GPIO.output(MOTOR_IN3, GPIO.LOW)
    GPIO.output(MOTOR_IN4, GPIO.HIGH)
    motor_pwm.ChangeDutyCycle(velocidad)


def motor_parar():
    motor_pwm.ChangeDutyCycle(0)
    GPIO.output(MOTOR_IN3, GPIO.LOW)
    GPIO.output(MOTOR_IN4, GPIO.LOW)


# ============================================================
# LIDAR D500
# ============================================================

PUERTO_LIDAR = "/dev/serial0"
BAUDRATE = 230400

try:
    lidar = serial.Serial(PUERTO_LIDAR, BAUDRATE, timeout=0.05)
    print("LIDAR conectado:", PUERTO_LIDAR)
except Exception as e:
    print("ERROR LIDAR:", e)
    lidar = None

puntos_lidar = []
lock_lidar = threading.Lock()


# ============================================================
# CRC-8
# ============================================================

def crc8(data):
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ 0x4D) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc


# ============================================================
# LECTURA LIDAR
# ============================================================

def leer_lidar():
    global puntos_lidar

    if lidar is None:
        return

    buffer = bytearray()

    while True:
        try:
            datos = lidar.read(256)
            if datos:
                buffer.extend(datos)

            while len(buffer) >= 47:
                posicion = buffer.find(b'\x54')

                if posicion < 0:
                    buffer.clear()
                    break

                if posicion > 0:
                    del buffer[:posicion]

                if len(buffer) < 47:
                    break

                paquete = buffer[:47]

                if paquete[1] != 0x2C:
                    del buffer[0]
                    continue

                if crc8(paquete[:46]) != paquete[46]:
                    del buffer[0]
                    continue

                angulo_inicio = int.from_bytes(paquete[4:6], byteorder="little") / 100.0
                angulo_final = int.from_bytes(paquete[42:44], byteorder="little") / 100.0

                if angulo_final >= angulo_inicio:
                    diferencia = angulo_final - angulo_inicio
                else:
                    diferencia = 360 - angulo_inicio + angulo_final

                nuevos_puntos = []

                for i in range(12):
                    indice = 6 + i * 3
                    distancia = int.from_bytes(paquete[indice:indice + 2], byteorder="little")
                    intensidad = paquete[indice + 2]
                    angulo = (angulo_inicio + diferencia * i / 11.0) % 360

                    if 50 <= distancia <= 12000:
                        nuevos_puntos.append((angulo, distancia, intensidad, time.time()))

                with lock_lidar:
                    puntos_lidar.extend(nuevos_puntos)
                    ahora = time.time()
                    puntos_lidar = [p for p in puntos_lidar if ahora - p[3] < 0.25]

                del buffer[:47]

        except Exception as e:
            print("Error LIDAR:", e)
            time.sleep(0.05)


# ============================================================
# ORIENTACION DEL LIDAR
# ============================================================

def lidar_a_pantalla(angulo):
    return (angulo - 90) % 360


def angulo_robot_a_lidar(angulo_robot):
    return (angulo_robot + 90) % 360


# ============================================================
# DISTANCIA POR ZONA
# ============================================================

def distancia_zona(angulo_centro, ancho=15, distancia_max=6000):
    objetivo = angulo_robot_a_lidar(angulo_centro)
    valores = []

    with lock_lidar:
        for (angulo, distancia, intensidad, tiempo_punto) in puntos_lidar:
            diferencia = abs((angulo - objetivo + 180) % 360 - 180)
            if diferencia <= ancho:
                if 50 <= distancia <= distancia_max:
                    valores.append(distancia)

    if not valores:
        return None

    valores.sort()
    if len(valores) >= 5:
        recortados = valores[1:-1]
    else:
        recortados = valores

    indice = max(0, int(len(recortados) * 0.20))
    return recortados[indice]


def distancia_frente():
    return distancia_zona(0, 25, 5000)


def distancia_izquierda():
    return distancia_zona(-90, 18, 5000)


def distancia_derecha():
    return distancia_zona(90, 18, 5000)


def distancia_frente_izquierda():
    return distancia_zona(-45, 20, 5000)


def distancia_frente_derecha():
    return distancia_zona(45, 20, 5000)


# ============================================================
# CAMARA / COLOR
# ============================================================

def detectar_color(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    rojo1 = cv2.inRange(hsv, np.array([0, 120, 80]), np.array([10, 255, 255]))
    rojo2 = cv2.inRange(hsv, np.array([170, 120, 80]), np.array([180, 255, 255]))
    mascara_roja = cv2.bitwise_or(rojo1, rojo2)

    mascara_verde = cv2.inRange(hsv, np.array([35, 80, 60]), np.array([90, 255, 255]))

    kernel = np.ones((5, 5), np.uint8)
    mascara_roja = cv2.morphologyEx(mascara_roja, cv2.MORPH_OPEN, kernel)
    mascara_roja = cv2.morphologyEx(mascara_roja, cv2.MORPH_CLOSE, kernel)
    mascara_verde = cv2.morphologyEx(mascara_verde, cv2.MORPH_OPEN, kernel)
    mascara_verde = cv2.morphologyEx(mascara_verde, cv2.MORPH_CLOSE, kernel)

    contornos_rojos, _ = cv2.findContours(mascara_roja, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contornos_verdes, _ = cv2.findContours(mascara_verde, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    area_roja = 0
    area_verde = 0

    if contornos_rojos:
        area_roja = cv2.contourArea(max(contornos_rojos, key=cv2.contourArea))

    if contornos_verdes:
        area_verde = cv2.contourArea(max(contornos_verdes, key=cv2.contourArea))

    if area_roja > 800 and area_roja > area_verde:
        return "ROJO"

    if area_verde > 800 and area_verde > area_roja:
        return "VERDE"

    return None


# ============================================================
# CONTROL DE DISTANCIA
# ============================================================

DISTANCIA_OBJETIVO_PARED = 550
DISTANCIA_RUTA_PARED = 550
TOLERANCIA_RUTA_PARED = 90
KP_RUTA_PARED = 0.022
MAX_CORRECCION_RUTA = 12
KP = 0.035
MAX_CORRECCION = 22

pared_preferida_ruta = None


def controlar_paredes():
    global pared_preferida_ruta

    izquierda = distancia_izquierda()
    derecha = distancia_derecha()
    pared_ruta = None

    if pared_preferida_ruta == "IZQUIERDA":
        pared_ruta = izquierda
    elif pared_preferida_ruta == "DERECHA":
        pared_ruta = derecha

    if pared_ruta is not None:
        error = pared_ruta - DISTANCIA_RUTA_PARED
        if abs(error) > TOLERANCIA_RUTA_PARED:
            correccion = error * KP_RUTA_PARED
            correccion = max(-MAX_CORRECCION_RUTA, min(MAX_CORRECCION_RUTA, correccion))

            if pared_preferida_ruta == "IZQUIERDA":
                objetivo = SERVO_CENTRO + correccion
            else:
                objetivo = SERVO_CENTRO - correccion

            establecer_objetivo_servo(objetivo)
            return

        establecer_objetivo_servo(SERVO_CENTRO)
        return

    if izquierda is None and derecha is None:
        establecer_objetivo_servo(SERVO_CENTRO)
        return

    if izquierda is not None and derecha is None:
        error = izquierda - DISTANCIA_OBJETIVO_PARED
        correccion = error * 0.025
        correccion = max(-MAX_CORRECCION, min(MAX_CORRECCION, correccion))
        establecer_objetivo_servo(SERVO_CENTRO - correccion)
        return

    if derecha is not None and izquierda is None:
        error = derecha - DISTANCIA_OBJETIVO_PARED
        correccion = error * 0.025
        correccion = max(-MAX_CORRECCION, min(MAX_CORRECCION, correccion))
        establecer_objetivo_servo(SERVO_CENTRO + correccion)
        return

    error = izquierda - derecha
    correccion = error * KP
    correccion = max(-MAX_CORRECCION, min(MAX_CORRECCION, correccion))
    establecer_objetivo_servo(SERVO_CENTRO + correccion)


# ============================================================
# REVERSA DE EMERGENCIA
# ============================================================

DISTANCIA_CRITICA = 200
DISTANCIA_GIRO = 830


def reversa_emergencia():
    print()
    print("!!!!!!!!!!!!!!!!!!!!!!!!")
    print(" PELIGRO - REVERSA")
    print("!!!!!!!!!!!!!!!!!!!!!!!!")

    motor_parar()
    time.sleep(0.10)
    establecer_objetivo_servo(SERVO_CENTRO)

    inicio_servo = time.monotonic()
    while time.monotonic() - inicio_servo < 0.25:
        actualizar_servo()
        time.sleep(0.01)

    print(">>> RETROCEDIENDO")
    motor_retroceder(90)

    inicio = time.monotonic()
    while time.monotonic() - inicio < 0.60:
        actualizar_servo()
        time.sleep(0.01)

    motor_parar()
    time.sleep(0.15)

    frente = distancia_frente()
    izquierda = distancia_izquierda()
    derecha = distancia_derecha()

    print("Despues de reversa:")
    print("F:", frente, "IZQ:", izquierda, "DER:", derecha)

    if izquierda is not None and derecha is not None:
        if izquierda < derecha:
            print("Acomodando hacia DERECHA")
            establecer_objetivo_servo(SERVO_DERECHA)
            motor_avanzar(80)

            inicio = time.monotonic()
            while time.monotonic() - inicio < 0.40:
                actualizar_servo()
                time.sleep(0.01)

        elif derecha < izquierda:
            print("Acomodando hacia IZQUIERDA")
            establecer_objetivo_servo(SERVO_IZQUIERDA)
            motor_avanzar(80)

            inicio = time.monotonic()
            while time.monotonic() - inicio < 0.40:
                actualizar_servo()
                time.sleep(0.01)

    establecer_objetivo_servo(SERVO_CENTRO)

    inicio = time.monotonic()
    while time.monotonic() - inicio < 0.35:
        actualizar_servo()
        time.sleep(0.01)

    motor_parar()
    print(">>> ACOMODADO")
    time.sleep(0.15)


# ============================================================
# EVASION POR COLOR
# ============================================================

def evitar_obstaculo(color):
    print("OBSTACULO")
    print("COLOR:", color)
    motor_avanzar(85)

    if color == "ROJO":
        establecer_objetivo_servo(SERVO_IZQUIERDA)
    elif color == "VERDE":
        establecer_objetivo_servo(SERVO_DERECHA)
    else:
        controlar_paredes()
        return

    inicio = time.monotonic()

    while True:
        actualizar_servo()
        frente = distancia_frente()

        if frente is None:
            break
        if frente > 1200:
            break
        if time.monotonic() - inicio > 3.0:
            break

        time.sleep(0.01)

    establecer_objetivo_servo(SERVO_CENTRO)


# ============================================================
# GIRO INTELIGENTE
# ============================================================

modo_giro = False
direccion_giro = None


# ============================================================
# MPU6050 - CONTROL DE 3 VUELTAS
# ============================================================

MPU6050_ADDR = 0x68
I2C_BUS = 1

MPU6050_PWR_MGMT_1 = 0x6B
MPU6050_GYRO_ZOUT_H = 0x47
GYRO_SCALE = 131.0

VUELTAS_OBJETIVO = 3
ANGULO_POR_VUELTA = 360.0
ANGULO_OBJETIVO = VUELTAS_OBJETIVO * ANGULO_POR_VUELTA

FACTOR_CORRECCION_GIRO = 1.02
INVERTIR_SIGNO_GYRO = False
ZONA_MUERTA_GYRO = 2.5
MAX_VELOCIDAD_GIRO = 300.0
DT_MAXIMO_MPU = 0.10
TIEMPO_ESPERA_PARADA = 2.5

mpu_bus = None
gyro_z_offset = 0.0
angulo_total = 0.0
ultimo_tiempo_mpu = None
limite_vueltas_alcanzado = False
tiempo_inicio_parada = None


def iniciar_mpu6050():
    global mpu_bus

    try:
        mpu_bus = smbus2.SMBus(I2C_BUS)
        mpu_bus.write_byte_data(MPU6050_ADDR, MPU6050_PWR_MGMT_1, 0x00)
        mpu_bus.write_byte_data(MPU6050_ADDR, 0x1B, 0x00)
        time.sleep(0.5)
        print("MPU6050 conectado correctamente.")
        return True
    except Exception as e:
        print("ERROR MPU6050:", e)
        mpu_bus = None
        return False


def leer_gyro_z():
    if mpu_bus is None:
        return 0.0

    try:
        datos = mpu_bus.read_i2c_block_data(MPU6050_ADDR, MPU6050_GYRO_ZOUT_H, 2)
        valor = (datos[0] << 8) | datos[1]

        if valor >= 32768:
            valor -= 65536

        velocidad = valor / GYRO_SCALE
        velocidad -= gyro_z_offset
        return velocidad

    except Exception as e:
        print("Error leyendo MPU6050:", e)
        return 0.0


def calibrar_mpu6050():
    global gyro_z_offset

    print()
    print("======================================")
    print(" CALIBRANDO MPU6050")
    print(" NO MUEVAS EL ROBOT")
    print("======================================")

    muestras = []
    inicio = time.monotonic()

    while time.monotonic() - inicio < 2.0:
        if mpu_bus is not None:
            try:
                datos = mpu_bus.read_i2c_block_data(MPU6050_ADDR, MPU6050_GYRO_ZOUT_H, 2)
                valor = (datos[0] << 8) | datos[1]

                if valor >= 32768:
                    valor -= 65536

                muestras.append(valor / GYRO_SCALE)
            except Exception:
                pass

        time.sleep(0.005)

    if muestras:
        gyro_z_offset = sum(muestras) / len(muestras)

    print("Offset gyro Z:", round(gyro_z_offset, 3), "deg/s")
    print("MPU6050 calibrado.")
    print()


def actualizar_mpu6050():
    global ultimo_tiempo_mpu
    global angulo_total

    if mpu_bus is None:
        return

    ahora = time.monotonic()

    if ultimo_tiempo_mpu is None:
        ultimo_tiempo_mpu = ahora
        return

    dt = ahora - ultimo_tiempo_mpu
    ultimo_tiempo_mpu = ahora

    if dt <= 0 or dt > DT_MAXIMO_MPU:
        return

    velocidad_z = leer_gyro_z()

    if abs(velocidad_z) < ZONA_MUERTA_GYRO:
        velocidad_z = 0.0

    velocidad_z = max(-MAX_VELOCIDAD_GIRO, min(MAX_VELOCIDAD_GIRO, velocidad_z))

    if INVERTIR_SIGNO_GYRO:
        velocidad_z = -velocidad_z

    grados = velocidad_z * dt
    grados *= FACTOR_CORRECCION_GIRO
    angulo_total += grados


def revisar_limite_vueltas():
    if limite_vueltas_alcanzado:
        return False

    if angulo_total >= ANGULO_OBJETIVO or angulo_total <= -ANGULO_OBJETIVO:
        return True

    return False


def detener_por_limite_giros():
    global modo_giro
    global limite_vueltas_alcanzado

    modo_giro = False
    limite_vueltas_alcanzado = True
    motor_parar()
    establecer_objetivo_servo(SERVO_CENTRO)

    inicio = time.monotonic()
    while time.monotonic() - inicio < 0.35:
        actualizar_servo()
        time.sleep(0.01)

    desactivar_servo()

    print()
    print("======================================")
    print(" 3 VUELTAS COMPLETADAS")
    print(" ANGULO:", round(angulo_total, 1), "grados")

    if angulo_total >= 0:
        print(" DIRECCION: IZQUIERDA (+)")
    else:
        print(" DIRECCION: DERECHA (-)")

    print(" ROBOT DETENIDO")
    print("======================================")
    print()


# ============================================================
# VARIABLES DE RUTA
# ============================================================

DISTANCIA_PELIGRO_LATERAL = 450
DISTANCIA_CENTRAR = 1000


def distancia_para_planificar(valor):
    if valor is None:
        return 5000
    return valor


def medir_sector(angulo, ancho=10, distancia_max=4500):
    return distancia_zona(angulo, ancho, distancia_max)


def valor_planificacion(valor):
    if valor is None:
        return 3500.0
    return float(valor)


def analizar_ruta():
    datos = {
        "izq_lateral": medir_sector(-90, 18),
        "der_lateral": medir_sector(90, 18),
        "izq_diag": medir_sector(-45, 16),
        "der_diag": medir_sector(45, 16),
        "izq_cerca": medir_sector(-25, 12),
        "der_cerca": medir_sector(25, 12),
        "frente": medir_sector(0, 25, 5000)
    }

    izq = (
        valor_planificacion(datos["izq_lateral"]) * 0.30
        + valor_planificacion(datos["izq_diag"]) * 0.35
        + valor_planificacion(datos["izq_cerca"]) * 0.35
    )

    der = (
        valor_planificacion(datos["der_lateral"]) * 0.30
        + valor_planificacion(datos["der_diag"]) * 0.35
        + valor_planificacion(datos["der_cerca"]) * 0.35
    )

    if datos["izq_cerca"] is not None and datos["izq_cerca"] < 600:
        izq -= (600 - datos["izq_cerca"]) * 1.8

    if datos["der_cerca"] is not None and datos["der_cerca"] < 600:
        der -= (600 - datos["der_cerca"]) * 1.8

    print(
        "ANALISIS ->",
        "IZQ:", round(izq),
        "DER:", round(der),
        "L:", datos["izq_lateral"],
        "D:", datos["der_lateral"],
        "DL:", datos["izq_diag"],
        "DR:", datos["der_diag"],
        "CL:", datos["izq_cerca"],
        "CR:", datos["der_cerca"]
    )

    if izq > der:
        return ("IZQUIERDA", izq, der, datos)

    return ("DERECHA", izq, der, datos)


def elegir_lado_mas_libre(izquierda, derecha):
    direccion, _, _, _ = analizar_ruta()
    return direccion


def girar_hacia_lado_abierto():
    global modo_giro
    global direccion_giro
    global pared_preferida_ruta

    izquierda = distancia_izquierda()
    derecha = distancia_derecha()
    frente = distancia_frente()

    if modo_giro:
        if direccion_giro == "IZQUIERDA":
            lado = izquierda
            contrario = derecha
        else:
            lado = derecha
            contrario = izquierda

        if lado is not None and lado < DISTANCIA_PELIGRO_LATERAL and (contrario is None or contrario > lado + 180):
            if direccion_giro == "IZQUIERDA":
                nueva = "DERECHA"
            else:
                nueva = "IZQUIERDA"

            print(">>> CAMINO ELEGIDO SE CERRO - CAMBIO A", nueva)
            direccion_giro = nueva

            if nueva == "IZQUIERDA":
                establecer_objetivo_servo(SERVO_DERECHA)
            else:
                establecer_objetivo_servo(SERVO_IZQUIERDA)

        if izquierda is not None and derecha is not None and frente is not None and frente >= DISTANCIA_CENTRAR:
            print(">>> PASILLO CONFIRMADO - CENTRANDO")
            establecer_objetivo_servo(SERVO_CENTRO)
            modo_giro = False
            motor_avanzar(82)
            return

        if frente is not None and frente < 650:
            motor_avanzar(70)
        elif frente is not None and frente < 950:
            motor_avanzar(74)
        else:
            motor_avanzar(80)

        return

    print()
    print(">>> ANALIZANDO RUTA COMPLETA")
    print("FRENTE:", frente, "IZQ:", izquierda, "DER:", derecha)

    direccion, score_izq, score_der, datos = analizar_ruta()

    if abs(score_izq - score_der) < 180:
        cerca_izq = valor_planificacion(datos["izq_cerca"])
        cerca_der = valor_planificacion(datos["der_cerca"])

        if cerca_izq > cerca_der + 100:
            direccion = "IZQUIERDA"
        elif cerca_der > cerca_izq + 100:
            direccion = "DERECHA"

    modo_giro = True
    direccion_giro = direccion
    pared_preferida_ruta = direccion

    if direccion == "IZQUIERDA":
        print(">>> RUTA FINAL: IZQUIERDA")
        print(">>> ENTRADA:", datos["izq_cerca"], "DIAGONAL:", datos["izq_diag"])
        establecer_objetivo_servo(SERVO_DERECHA)
    else:
        print(">>> RUTA FINAL: DERECHA")
        print(">>> ENTRADA:", datos["der_cerca"], "DIAGONAL:", datos["der_diag"])
        establecer_objetivo_servo(SERVO_IZQUIERDA)

    motor_avanzar(72)


# ============================================================
# RADAR 360°
# ============================================================

RADAR_TAMANO = 400
radar = np.zeros((RADAR_TAMANO, RADAR_TAMANO, 3), dtype=np.uint8)


def dibujar_radar():
    global radar
    radar[:] = 0
    centro = RADAR_TAMANO // 2

    for metros in [0.5, 1, 1.5, 2, 3, 4]:
        radio = int(metros * 100)
        cv2.circle(radar, (centro, centro), radio, (45, 45, 45), 1)
        cv2.putText(radar, str(metros) + "m", (centro + radio + 3, centro), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (120, 120, 120), 1)

    cv2.line(radar, (centro, 0), (centro, RADAR_TAMANO), (40, 40, 40), 1)
    cv2.line(radar, (0, centro), (RADAR_TAMANO, centro), (40, 40, 40), 1)

    with lock_lidar:
        copia = list(puntos_lidar)

    ahora = time.time()

    for (angulo, distancia, intensidad, tiempo_punto) in copia:
        if ahora - tiempo_punto > 0.65:
            continue

        metros = distancia / 1000.0
        if metros > 4:
            continue

        angulo_pantalla = lidar_a_pantalla(angulo)
        rad = math.radians(angulo_pantalla)

        x = int(centro + math.sin(rad) * metros * 100)
        y = int(centro - math.cos(rad) * metros * 100)

        if 0 <= x < RADAR_TAMANO and 0 <= y < RADAR_TAMANO:
            cv2.circle(radar, (x, y), 2, (255, 255, 255), -1)

    cv2.circle(radar, (centro, centro), 8, (255, 255, 255), -1)
    cv2.putText(radar, "FRENTE", (centro - 35, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    cv2.putText(radar, "ATRAS", (centro - 25, RADAR_TAMANO - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    cv2.putText(radar, "IZQ", (10, centro), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    cv2.putText(radar, "DER", (RADAR_TAMANO - 40, centro), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)


# ============================================================
# HILO LIDAR
# ============================================================

if lidar is not None:
    hilo_lidar = threading.Thread(target=leer_lidar, daemon=True)
    hilo_lidar.start()


# ============================================================
# INICIAR MPU6050
# ============================================================

mpu_ok = iniciar_mpu6050()

if mpu_ok:
    calibrar_mpu6050()
    ultimo_tiempo_mpu = time.monotonic()
else:
    print("ADVERTENCIA: MPU6050 NO DISPONIBLE")


# ============================================================
# CAMARA
# ============================================================

picam2 = Picamera2()
config = picam2.create_preview_configuration(
    main={
        "size": (640, 480),
        "format": "RGB888"
    }
)
picam2.configure(config)
picam2.start()
time.sleep(2)

FRANJA_Y_INICIO = 180
FRANJA_Y_FINAL = 310


# ============================================================
# ESPERAR BOTON
# ============================================================

print()
print("======================================")
print(" ROBOT AUTONOMO")
print(" SERVO 180° SUAVE")
print(" MOTOR 90%")
print(" MPU6050 - 3 VUELTAS +/-1080°")
print(" IZQUIERDA = POSITIVO")
print(" DERECHA = NEGATIVO")
print("======================================")
print("Servo apagado hasta pulsar boton.")
print()

motor_parar()
desactivar_servo()
GPIO.output(LED_VERDE, GPIO.HIGH)

while GPIO.input(BOTON) == GPIO.HIGH:
    frame = picam2.capture_array()
    frame = cv2.rotate(frame, cv2.ROTATE_180)

    franja = frame[FRANJA_Y_INICIO:FRANJA_Y_FINAL, 0:640]
    color = detectar_color(franja)
    camara_bgr = cv2.cvtColor(franja, cv2.COLOR_RGB2BGR)

    cv2.putText(camara_bgr, "ANGULO: %.1f deg" % angulo_total, (20, 75), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
    cv2.putText(camara_bgr, "VUELTAS: %.2f / %d" % (abs(angulo_total) / 360.0, VUELTAS_OBJETIVO), (20, 110), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

    cv2.imshow("CAMARA", camara_bgr)
    dibujar_radar()
    cv2.imshow("LIDAR 360", radar)

    if cv2.waitKey(1) & 0xFF == 27:
        picam2.stop()
        servo_pwm.stop()
        motor_pwm.stop()
        GPIO.cleanup()
        cv2.destroyAllWindows()
        raise SystemExit

    time.sleep(0.01)


# ============================================================
# ARRANQUE
# ============================================================

print()
print("BOTON PRESIONADO")
print("INICIANDO ROBOT...")

angulo_total = 0.0
limite_vueltas_alcanzado = False
tiempo_inicio_parada = None
ultimo_tiempo_mpu = time.monotonic()

activar_servo()
servo_angulo_actual = SERVO_CENTRO
servo_objetivo = SERVO_CENTRO
time.sleep(0.20)
motor_avanzar(VELOCIDAD)


# ============================================================
# LOOP PRINCIPAL
# ============================================================

ultimo_mensaje = 0

try:
    while True:
        actualizar_mpu6050()

        if revisar_limite_vueltas():
            if tiempo_inicio_parada is None:
                tiempo_inicio_parada = time.monotonic()
                print()
                print(">>> LIMITE DE 1080 GRADOS DETECTADO")
                print(">>> ANGULO:", round(angulo_total, 1), "grados")

                if angulo_total > 0:
                    print(">>> GIRO ACUMULADO: IZQUIERDA (+)")
                else:
                    print(">>> GIRO ACUMULADO: DERECHA (-)")

                print(">>> 3 VUELTAS COMPLETADAS")
                print(">>> MOTOR DETENIDO INMEDIATAMENTE")
                print(">>> ANALIZANDO DURANTE 2.5 SEGUNDOS")

                motor_parar()
                establecer_objetivo_servo(SERVO_CENTRO)

        if tiempo_inicio_parada is not None:
            tiempo_transcurrido = time.monotonic() - tiempo_inicio_parada
            if tiempo_transcurrido >= TIEMPO_ESPERA_PARADA:
                detener_por_limite_giros()
                break

        frame = picam2.capture_array()
        frame = cv2.rotate(frame, cv2.ROTATE_180)
        franja = frame[FRANJA_Y_INICIO:FRANJA_Y_FINAL, 0:640]
        color = detectar_color(franja)

        frente = distancia_frente()
        izquierda = distancia_izquierda()
        derecha = distancia_derecha()

        ahora = time.time()
        if ahora - ultimo_mensaje > 0.3:
            print(
                "F:", frente,
                "IZQ:", izquierda,
                "DER:", derecha,
                "COLOR:", color,
                "VUELTAS:", round(abs(angulo_total) / 360.0, 2),
                "/", VUELTAS_OBJETIVO,
                "ANG:", round(angulo_total, 1), "deg",
                "SERVO:", round(servo_angulo_actual, 1)
            )
            ultimo_mensaje = ahora

        if tiempo_inicio_parada is not None:
            actualizar_servo()
        else:
            if frente is not None and frente <= DISTANCIA_CRITICA:
                reversa_emergencia()
                motor_avanzar(VELOCIDAD)
            elif modo_giro or (frente is not None and frente <= DISTANCIA_GIRO):
                girar_hacia_lado_abierto()
            else:
                motor_avanzar(VELOCIDAD)
                controlar_paredes()

            actualizar_servo()

        camara_bgr = cv2.cvtColor(franja, cv2.COLOR_RGB2BGR)

        if color == "ROJO":
            cv2.putText(camara_bgr, "ROJO", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        elif color == "VERDE":
            cv2.putText(camara_bgr, "VERDE", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        cv2.putText(camara_bgr, "ANGULO: %.1f deg" % angulo_total, (20, 75), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        cv2.putText(camara_bgr, "VUELTAS: %.2f / %d" % (abs(angulo_total) / 360.0, VUELTAS_OBJETIVO), (20, 110), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

        cv2.imshow("CAMARA", camara_bgr)
        dibujar_radar()
        cv2.imshow("LIDAR 360", radar)

        tecla = cv2.waitKey(1) & 0xFF
        if tecla == 27:
            break

        time.sleep(0.005)


# ============================================================
# APAGADO
# ============================================================

except KeyboardInterrupt:
    print("Programa detenido.")

finally:
    print("Apagando robot...")
    motor_parar()
    establecer_objetivo_servo(SERVO_CENTRO)

    inicio = time.monotonic()
    while time.monotonic() - inicio < 0.5:
        actualizar_servo()
        time.sleep(0.01)

    desactivar_servo()
    servo_pwm.stop()
    motor_pwm.stop()
    picam2.stop()

    if mpu_bus is not None:
        try:
            mpu_bus.close()
        except Exception:
            pass

    GPIO.cleanup()
    cv2.destroyAllWindows()
    print("Robot apagado.")
