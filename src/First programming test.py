from picamera2 import Picamera2
import cv2
import numpy as np
import time
import RPi.GPIO as GPIO
import serial
import threading
import math
import os

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

# Led verde - robot listo
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

SERVO_CENTRO = 88
SERVO_IZQUIERDA = 60
SERVO_DERECHA = 120

SERVO_MIN = 50
SERVO_MAX = 130

INVERTIR_SERVO = False

# ------------------------------------------------------------
# SERVO DE RESPUESTA DIRECTA
# ------------------------------------------------------------
# El servo ya NO avanza por pasos.
# Cada vez que se cambia el objetivo, se manda inmediatamente
# el angulo completo al servo.
#
# Esto evita el pequeno giro contrario que ocurria mientras el
# programa todavia estaba ejecutando el objetivo anterior.

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
        servo_pwm.ChangeDutyCycle(
            2.5 + (SERVO_CENTRO / 180.0) * 10.0
        )

        servo_activo = True
        ultimo_movimiento_servo = time.monotonic()
        time.sleep(0.15)


def desactivar_servo():
    global servo_activo

    if servo_activo:
        servo_pwm.ChangeDutyCycle(0)
        servo_activo = False


# ------------------------------------------------------------
# CONTROL DE SERVO SIN TIRONES
# ------------------------------------------------------------
# El objetivo se cambia de inmediato, pero el servo fisicamente
# se acerca al objetivo con una rampa rapida. Esto evita que un
# cambio brusco de 88 -> 120 -> 88 provoque tirones.
SERVO_PASO_RAPIDO = 4.0
SERVO_PASO_MEDIO = 2.5
SERVO_PASO_FIN = 1.2
SERVO_INTERVALO = 0.015


def establecer_objetivo_servo(angulo):
    global servo_objetivo

    servo_objetivo = max(
        SERVO_MIN,
        min(SERVO_MAX, float(angulo))
    )


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
        # Mucho recorrido = movimiento rapido.
        # Cerca del objetivo = movimiento mas fino.
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

    servo_angulo_actual = max(
        SERVO_MIN,
        min(SERVO_MAX, servo_angulo_actual)
    )

    salida = servo_angulo_actual

    if INVERTIR_SERVO:
        salida = 180 - salida

    duty = 2.5 + (salida / 180.0) * 10.0
    servo_pwm.ChangeDutyCycle(duty)
    ultimo_movimiento_servo = ahora


def mover_servo_suave(angulo):
    # Conservamos el nombre para no romper otras partes del programa,
    # pero ahora el movimiento es directo.
    establecer_objetivo_servo(angulo)


# ============================================================
# MOTOR
# ============================================================

motor_pwm = GPIO.PWM(
    MOTOR_EN,
    1000
)

motor_pwm.start(0)

# VELOCIDAD NORMAL
VELOCIDAD = 89


def motor_avanzar(velocidad=VELOCIDAD):
    GPIO.output(
        MOTOR_IN3,
        GPIO.HIGH
    )

    GPIO.output(
        MOTOR_IN4,
        GPIO.LOW
    )

    motor_pwm.ChangeDutyCycle(
        velocidad
    )


def motor_retroceder(velocidad=90):
    GPIO.output(
        MOTOR_IN3,
        GPIO.LOW
    )

    GPIO.output(
        MOTOR_IN4,
        GPIO.HIGH
    )

    motor_pwm.ChangeDutyCycle(
        velocidad
    )


def motor_parar():
    motor_pwm.ChangeDutyCycle(0)

    GPIO.output(
        MOTOR_IN3,
        GPIO.LOW
    )

    GPIO.output(
        MOTOR_IN4,
        GPIO.LOW
    )


# ============================================================
# LIDAR D500
# ============================================================

PUERTO_LIDAR = "/dev/serial0"
BAUDRATE = 230400

try:
    lidar = serial.Serial(
        PUERTO_LIDAR,
        BAUDRATE,
        timeout=0.05
    )

    print(
        "LIDAR conectado:",
        PUERTO_LIDAR
    )

except Exception as e:
    print(
        "ERROR LIDAR:",
        e
    )

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
                crc = (
                    (crc << 1)
                    ^ 0x4D
                ) & 0xFF

            else:
                crc = (
                    crc << 1
                ) & 0xFF

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
                posicion = buffer.find(
                    b'\x54'
                )

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

                if crc8(
                    paquete[:46]
                ) != paquete[46]:
                    del buffer[0]
                    continue

                angulo_inicio = (
                    int.from_bytes(
                        paquete[4:6],
                        byteorder="little"
                    ) / 100.0
                )

                angulo_final = (
                    int.from_bytes(
                        paquete[42:44],
                        byteorder="little"
                    ) / 100.0
                )

                if angulo_final >= angulo_inicio:
                    diferencia = (
                        angulo_final
                        - angulo_inicio
                    )

                else:
                    diferencia = (
                        360
                        - angulo_inicio
                        + angulo_final
                    )

                nuevos_puntos = []

                for i in range(12):
                    indice = 6 + i * 3

                    distancia = int.from_bytes(
                        paquete[
                            indice:
                            indice + 2
                        ],
                        byteorder="little"
                    )

                    intensidad = paquete[
                        indice + 2
                    ]

                    angulo = (
                        angulo_inicio
                        + diferencia
                        * i
                        / 11.0
                    ) % 360

                    if (
                        50
                        <= distancia
                        <= 12000
                    ):
                        nuevos_puntos.append(
                            (
                                angulo,
                                distancia,
                                intensidad,
                                time.time()
                            )
                        )

                with lock_lidar:
                    puntos_lidar.extend(
                        nuevos_puntos
                    )

                    ahora = time.time()

                    puntos_lidar = [
                        p
                        for p in puntos_lidar
                        if ahora - p[3] < 0.25
                    ]

                del buffer[:47]

        except Exception as e:
            print(
                "Error LIDAR:",
                e
            )

            time.sleep(0.05)


# ============================================================
# ORIENTACION DEL LIDAR
# ============================================================

def lidar_a_pantalla(angulo):
    # FRENTE = 0
    # DERECHA = 90
    # ATRAS = 180
    # IZQUIERDA = 270

    return (
        angulo - 90
    ) % 360


def angulo_robot_a_lidar(angulo_robot):
    return (
        angulo_robot + 90
    ) % 360


# ============================================================
# DISTANCIA POR ZONA
# ============================================================

def distancia_zona(
    angulo_centro,
    ancho=15,
    distancia_max=6000
):
    objetivo = (
        angulo_robot_a_lidar(
            angulo_centro
        )
    )

    valores = []

    with lock_lidar:
        for (
            angulo,
            distancia,
            intensidad,
            tiempo_punto
        ) in puntos_lidar:
            diferencia = abs(
                (
                    angulo
                    - objetivo
                    + 180
                ) % 360
                - 180
            )

            if diferencia <= ancho:
                if (
                    50
                    <= distancia
                    <= distancia_max
                ):
                    valores.append(
                        distancia
                    )

    if not valores:
        return None

    # Medida robusta: quitamos extremos y usamos un percentil bajo.
    # Esto evita que un unico rebote del LiDAR haga que el robot
    # cambie de direccion de forma brusca.
    valores.sort()

    if len(valores) >= 5:
        recortados = valores[1:-1]
    else:
        recortados = valores

    indice = max(
        0,
        int(len(recortados) * 0.20)
    )

    return recortados[indice]


def distancia_frente():
    # Zona frontal mas amplia para detectar la pared antes
    # y comenzar el giro con anticipacion.
    return distancia_zona(
        0,
        25,
        5000
    )


def distancia_izquierda():
    return distancia_zona(
        -90,
        18,
        5000
    )


def distancia_derecha():
    return distancia_zona(
        90,
        18,
        5000
    )


def distancia_frente_izquierda():
    return distancia_zona(
        -45,
        20,
        5000
    )


def distancia_frente_derecha():
    return distancia_zona(
        45,
        20,
        5000
    )


# ============================================================
# CAMARA / COLOR
# ============================================================

def detectar_color(frame):
    hsv = cv2.cvtColor(
        frame,
        cv2.COLOR_BGR2HSV
    )

    # ROJO
    rojo1 = cv2.inRange(
        hsv,
        np.array([0, 120, 80]),
        np.array([10, 255, 255])
    )

    rojo2 = cv2.inRange(
        hsv,
        np.array([170, 120, 80]),
        np.array([180, 255, 255])
    )

    mascara_roja = cv2.bitwise_or(
        rojo1,
        rojo2
    )

    # VERDE
    mascara_verde = cv2.inRange(
        hsv,
        np.array([35, 80, 60]),
        np.array([90, 255, 255])
    )

    kernel = np.ones(
        (5, 5),
        np.uint8
    )

    mascara_roja = cv2.morphologyEx(
        mascara_roja,
        cv2.MORPH_OPEN,
        kernel
    )

    mascara_roja = cv2.morphologyEx(
        mascara_roja,
        cv2.MORPH_CLOSE,
        kernel
    )

    mascara_verde = cv2.morphologyEx(
        mascara_verde,
        cv2.MORPH_OPEN,
        kernel
    )

    mascara_verde = cv2.morphologyEx(
        mascara_verde,
        cv2.MORPH_CLOSE,
        kernel
    )

    contornos_rojos, _ = cv2.findContours(
        mascara_roja,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE
    )

    contornos_verdes, _ = cv2.findContours(
        mascara_verde,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE
    )

    area_roja = 0
    area_verde = 0

    if contornos_rojos:
        area_roja = cv2.contourArea(
            max(
                contornos_rojos,
                key=cv2.contourArea
            )
        )

    if contornos_verdes:
        area_verde = cv2.contourArea(
            max(
                contornos_verdes,
                key=cv2.contourArea
            )
        )

    if (
        area_roja > 800
        and area_roja > area_verde
    ):
        return "ROJO"

    if (
        area_verde > 800
        and area_verde > area_roja
    ):
        return "VERDE"

    return None


# ============================================================
# CONTROL DE DISTANCIA
# ============================================================

# Distancia deseada de las paredes
# 500 mm = 50 cm
DISTANCIA_OBJETIVO_PARED = 550

# Seguimiento de la pared del lado elegido en la ruta.
# El robot intenta pasar relativamente cerca de ella sin pegarse.
DISTANCIA_RUTA_PARED = 550
TOLERANCIA_RUTA_PARED = 90
KP_RUTA_PARED = 0.022
MAX_CORRECCION_RUTA = 12

KP = 0.035
MAX_CORRECCION = 22


def controlar_paredes():
    global pared_preferida_ruta

    izquierda = distancia_izquierda()
    derecha = distancia_derecha()

    # --------------------------------------------------------
    # SEGUIMIENTO SUAVE DE LA PARED DE LA RUTA
    # --------------------------------------------------------
    pared_ruta = None

    if pared_preferida_ruta == "IZQUIERDA":
        pared_ruta = izquierda
    elif pared_preferida_ruta == "DERECHA":
        pared_ruta = derecha

    if pared_ruta is not None:
        error = pared_ruta - DISTANCIA_RUTA_PARED

        if abs(error) > TOLERANCIA_RUTA_PARED:
            correccion = error * KP_RUTA_PARED
            correccion = max(
                -MAX_CORRECCION_RUTA,
                min(MAX_CORRECCION_RUTA, correccion)
            )

            # Comandos fisicos: 120 = izquierda, 60 = derecha.
            if pared_preferida_ruta == "IZQUIERDA":
                objetivo = SERVO_CENTRO + correccion
            else:
                objetivo = SERVO_CENTRO - correccion

            establecer_objetivo_servo(objetivo)
            return

        # Dentro de la tolerancia dejamos el servo casi centrado.
        establecer_objetivo_servo(SERVO_CENTRO)
        return

    if (
        izquierda is None
        and derecha is None
    ):
        establecer_objetivo_servo(
            SERVO_CENTRO
        )

        return

    # --------------------------------------------------------
    # SOLO IZQUIERDA
    # --------------------------------------------------------

    if (
        izquierda is not None
        and derecha is None
    ):
        error = (
            izquierda
            - DISTANCIA_OBJETIVO_PARED
        )

        correccion = (
            error * 0.025
        )

        correccion = max(
            -MAX_CORRECCION,
            min(
                MAX_CORRECCION,
                correccion
            )
        )

        establecer_objetivo_servo(
            SERVO_CENTRO
            - correccion
        )

        return

    # --------------------------------------------------------
    # SOLO DERECHA
    # --------------------------------------------------------

    if (
        derecha is not None
        and izquierda is None
    ):
        error = (
            derecha
            - DISTANCIA_OBJETIVO_PARED
        )

        correccion = (
            error * 0.025
        )

        correccion = max(
            -MAX_CORRECCION,
            min(
                MAX_CORRECCION,
                correccion
            )
        )

        establecer_objetivo_servo(
            SERVO_CENTRO
            + correccion
        )

        return

    # --------------------------------------------------------
    # DOS PAREDES
    # --------------------------------------------------------

    error = (
        izquierda
        - derecha
    )

    correccion = (
        error * KP
    )

    correccion = max(
        -MAX_CORRECCION,
        min(
            MAX_CORRECCION,
            correccion
        )
    )

    establecer_objetivo_servo(
        SERVO_CENTRO
        + correccion
    )


# ============================================================
# REVERSA DE EMERGENCIA
# ============================================================

DISTANCIA_CRITICA = 100
DISTANCIA_GIRO = 800


def reversa_emergencia():
    print()
    print(
        "!!!!!!!!!!!!!!!!!!!!!!!!"
    )
    print(
        " PELIGRO - REVERSA"
    )
    print(
        "!!!!!!!!!!!!!!!!!!!!!!!!"
    )

    # --------------------------------------------------------
    # PRIMERO PARAR
    # --------------------------------------------------------

    motor_parar()

    time.sleep(0.10)

    # --------------------------------------------------------
    # ENDEREZAR SERVO
    # --------------------------------------------------------

    establecer_objetivo_servo(
        SERVO_CENTRO
    )

    # Dar tiempo al servo para comenzar a centrarse
    inicio_servo = time.monotonic()

    while (
        time.monotonic()
        - inicio_servo
        < 0.25
    ):
        actualizar_servo()
        time.sleep(0.01)

    # --------------------------------------------------------
    # AHORA SÍ: REVERSA REAL
    # --------------------------------------------------------

    print(
        ">>> RETROCEDIENDO"
    )

    motor_retroceder(90)

    inicio = time.monotonic()

    while (
        time.monotonic()
        - inicio
        < 0.60
    ):
        actualizar_servo()

        time.sleep(0.01)

    # --------------------------------------------------------
    # PARAR DESPUÉS DE REVERSA
    # --------------------------------------------------------

    motor_parar()

    time.sleep(0.15)

    # --------------------------------------------------------
    # REVISAR ALREDEDOR
    # --------------------------------------------------------

    frente = distancia_frente()
    izquierda = distancia_izquierda()
    derecha = distancia_derecha()

    print(
        "Después de reversa:"
    )

    print(
        "F:",
        frente,
        "IZQ:",
        izquierda,
        "DER:",
        derecha
    )

    # --------------------------------------------------------
    # ACOMODARSE HACIA EL LADO MÁS LIBRE
    # --------------------------------------------------------

    if (
        izquierda is not None
        and derecha is not None
    ):
        if izquierda < derecha:
            print(
                "Acomodando hacia DERECHA"
            )

            establecer_objetivo_servo(
                SERVO_DERECHA
            )

            motor_avanzar(80)

            inicio = time.monotonic()

            while (
                time.monotonic()
                - inicio
                < 0.40
            ):
                actualizar_servo()
                time.sleep(0.01)

        elif derecha < izquierda:
            print(
                "Acomodando hacia IZQUIERDA"
            )

            establecer_objetivo_servo(
                SERVO_IZQUIERDA
            )

            motor_avanzar(80)

            inicio = time.monotonic()

            while (
                time.monotonic()
                - inicio
                < 0.40
            ):
                actualizar_servo()
                time.sleep(0.01)

    # --------------------------------------------------------
    # CENTRAR
    # --------------------------------------------------------

    establecer_objetivo_servo(
        SERVO_CENTRO
    )

    inicio = time.monotonic()

    while (
        time.monotonic()
        - inicio
        < 0.35
    ):
        actualizar_servo()
        time.sleep(0.01)

    motor_parar()

    print(
        ">>> ACOMODADO"
    )

    time.sleep(0.15)


# ============================================================
# EVASION POR COLOR
# ============================================================

def evitar_obstaculo(color):
    print(
        "OBSTACULO"
    )

    print(
        "COLOR:",
        color
    )

    motor_avanzar(85)

    # ROJO -> IZQUIERDA
    if color == "ROJO":
        establecer_objetivo_servo(
            SERVO_IZQUIERDA
        )

    # VERDE -> DERECHA
    elif color == "VERDE":
        establecer_objetivo_servo(
            SERVO_DERECHA
        )

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

        if (
            time.monotonic()
            - inicio
            > 3.0
        ):
            break

        time.sleep(0.01)

    establecer_objetivo_servo(
        SERVO_CENTRO
    )


# ============================================================
# GIRO INTELIGENTE HACIA LA RUTA ABIERTA
# ============================================================

modo_giro = False
direccion_giro = None

# ============================================================
# CONTADOR DE GIROS
# ============================================================
MAX_GIROS = 12
giros_realizados = 0
limite_giros_alcanzado = False

pared_preferida_ruta = None
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
        "frente": medir_sector(0, 25, 5000),
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
        return "IZQUIERDA", izq, der, datos

    return "DERECHA", izq, der, datos


def elegir_lado_mas_libre(izquierda, derecha):
    direccion, _, _, _ = analizar_ruta()
    return direccion


def detener_por_limite_giros():
    global modo_giro
    global limite_giros_alcanzado

    modo_giro = False
    motor_parar()
    establecer_objetivo_servo(SERVO_CENTRO)

    inicio = time.monotonic()
    while time.monotonic() - inicio < 0.35:
        actualizar_servo()
        time.sleep(0.01)

    desactivar_servo()
    limite_giros_alcanzado = True

    print()
    print("======================================")
    print(" LIMITE DE 12 GIROS ALCANZADO")
    print(" ROBOT DETENIDO")
    print("======================================")
    print()


def girar_hacia_lado_abierto():
    global modo_giro
    global direccion_giro
    global pared_preferida_ruta
    global giros_realizados
    global limite_giros_alcanzado

    izquierda = distancia_izquierda()
    derecha = distancia_derecha()
    frente = distancia_frente()

    # --------------------------------------------------------
    # SI YA ESTAMOS GIRANDO
    # --------------------------------------------------------
    if modo_giro:
        lado = izquierda if direccion_giro == "IZQUIERDA" else derecha
        contrario = derecha if direccion_giro == "IZQUIERDA" else izquierda

        if (
            lado is not None
            and lado < DISTANCIA_PELIGRO_LATERAL
            and (
                contrario is None
                or contrario > lado + 180
            )
        ):
            nueva = "DERECHA" if direccion_giro == "IZQUIERDA" else "IZQUIERDA"
            print(">>> CAMINO ELEGIDO SE CERRO - CAMBIO A", nueva)
            direccion_giro = nueva

            if nueva == "IZQUIERDA":
                establecer_objetivo_servo(SERVO_DERECHA)
            else:
                establecer_objetivo_servo(SERVO_IZQUIERDA)

        if (
            izquierda is not None
            and derecha is not None
            and frente is not None
            and frente >= DISTANCIA_CENTRAR
        ):
            print(">>> PASILLO CONFIRMADO - CENTRANDO")
            establecer_objetivo_servo(SERVO_CENTRO)
            modo_giro = False

            if limite_giros_alcanzado:
                detener_por_limite_giros()
                return

            motor_avanzar(82)
            return

        if frente is not None and frente < 650:
            motor_avanzar(70)
        elif frente is not None and frente < 950:
            motor_avanzar(74)
        else:
            motor_avanzar(80)

        return

    # --------------------------------------------------------
    # NUEVO GIRO: ANALISIS MULTISECTOR
    # --------------------------------------------------------

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

    # --------------------------------------------------------
    # CONTAR EL NUEVO GIRO
    # --------------------------------------------------------
    giros_realizados += 1
    print(">>> GIRO #", giros_realizados, "DE", MAX_GIROS, "->", direccion)

    if giros_realizados >= MAX_GIROS:
        limite_giros_alcanzado = True

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


# ===========================================================
# RADAR 360°
# ============================================================

RADAR_TAMANO = 400

radar = np.zeros(
    (
        RADAR_TAMANO,
        RADAR_TAMANO,
        3
    ),
    dtype=np.uint8
)


def dibujar_radar():
    global radar

    radar[:] = 0

    centro = (
        RADAR_TAMANO // 2
    )

    # ANILLOS
    for metros in [
        0.5,
        1,
        1.5,
        2,
        3,
        4
    ]:
        radio = int(
            metros * 100
        )

        cv2.circle(
            radar,
            (
                centro,
                centro
            ),
            radio,
            (45, 45, 45),
            1
        )

        cv2.putText(
            radar,
            str(metros) + "m",
            (
                centro
                + radio
                + 3,
                centro
            ),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.4,
            (120, 120, 120),
            1
        )

    # EJES
    cv2.line(
        radar,
        (
            centro,
            0
        ),
        (
            centro,
            RADAR_TAMANO
        ),
        (45, 45, 45),
        1
    )

    cv2.line(
        radar,
        (
            0,
            centro
        ),
        (
            RADAR_TAMANO,
            centro
        ),
        (45, 45, 45),
        1
    )
