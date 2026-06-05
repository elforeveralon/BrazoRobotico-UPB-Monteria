import time
import math
import numpy as np
import cv2
from picamera2 import Picamera2
from gpiozero import OutputDevice
from datetime import datetime

# ── GPIO Motores ──────────────────────────────────────────────────
step_base   = OutputDevice(17)
dir_base    = OutputDevice(27)
step_hombro = OutputDevice(22)
dir_hombro  = OutputDevice(23)
step_codo   = OutputDevice(24)
dir_codo    = OutputDevice(25)

# ── Parámetros del brazo ──────────────────────────────────────────
Z_HOMBRO = 105

L1  = 132.42
L2a = 220
L2b = 80

OFFSET_Y = 185
ANCHO    = 345

Z_SUPERFICIE = 0

HOME_T1 = 0.0
HOME_T2 = 90.0
HOME_T3 = 10.0

PPV_BASE   = 15760
PPV_HOMBRO = 16480
PPV_CODO   = 19029
RETARDO_PULSO = 0.002

pasos_actuales = [0, 0, 0]

# ── Esquinas ROI ──────────────────────────────────────────────────
esquinas = np.array([
    [460,  1332],
    [2572, 20],
    [3180, 28],
    [4600, 860],
    [4604, 1572],
    [3680, 2580],
    [1324, 2580]
], dtype=np.int32)

# ── Homografía ────────────────────────────────────────────────────
H = np.load("homografia.npy")

def pixel_a_mm(px, py):
    punto = np.float32([[px, py]])
    return cv2.perspectiveTransform(punto.reshape(1,1,2), H)[0][0]

# ── Sistema de coordenadas ────────────────────────────────────────
def camara_a_brazo(x_mm, y_mm):
    x = (ANCHO / 2) - x_mm
    y = OFFSET_Y + y_mm 
    z = Z_SUPERFICIE
    return x, y, z

# ── Cinemática inversa ────────────────────────────────────────────
# Convención: θ2 medido desde vertical (0°=vertical, crece alejándose)
#             θ3 medido desde L1 (0°=alineado, negativo=abajo)
def cinematica_inversa(x, y, z):

    theta1 = math.degrees(
        math.atan2(x, y)
    )

    r = math.sqrt(
        x**2 +
        y**2
    )

    z_muneca = -(Z_HOMBRO - L2b)

    d = math.sqrt(
        r**2 +
        z_muneca**2
    )

    if d > (L1 + L2a):
        print(f"Fuera de alcance: {d:.1f} mm")
        return None

    if d < abs(L1 - L2a):
        print(f"Demasiado cerca: {d:.1f} mm")
        return None

    cos_t3_int = (
        (L1**2 + L2a**2 - d**2)
        /
        (2 * L1 * L2a)
    )

    cos_t3_int = max(
        -1.0,
        min(1.0, cos_t3_int)
    )

    theta3_interior = math.degrees(
        math.acos(cos_t3_int)
    )

    theta3 = 180.0 - theta3_interior

    print(f"theta3 calculado = {theta3}")
    alpha = math.degrees(
        math.atan2(
            25.0,
            r
        )
    )

    cos_beta = (
        (L1**2 + d**2 - L2a**2)
        /
        (2 * L1 * d)
    )

    cos_beta = max(
        -1.0,
        min(1.0, cos_beta)
    )

    beta = math.degrees(
        math.acos(cos_beta)
    )

    theta2 = alpha + beta

    if theta2 < -20.0 or theta2 > 90.0:
        print(f"θ2 fuera de rango: {theta2:.2f}")
        return None

    if theta3 < -138.4 or theta3 > 138.4:
        print(f"θ3 fuera de rango: {theta3:.2f}")
        return None

    return (
        theta1,
        theta2,
        theta3
    )
# ── Movimiento ────────────────────────────────────────────────────
def ir_a_angulos(t1, t2, t3):

    delta_t1 = -(t1 - HOME_T1)
    delta_t2 = -(t2 - HOME_T2)
    delta_t3 = +(t3 - HOME_T3)

    p1 = int((delta_t1 / 360.0) * PPV_BASE)
    p2 = int((delta_t2 / 360.0) * PPV_HOMBRO)
    p3 = int((delta_t3 / 360.0) * PPV_CODO)

    mover1 = p1 - pasos_actuales[0]
    mover2 = p2 - pasos_actuales[1]
    mover3 = p3 - pasos_actuales[2]

    print(f"θ1={t1:.2f}° -> {p1} pasos")
    print(f"θ2={t2:.2f}° -> {p2} pasos")
    print(f"θ3={t3:.2f}° -> {p3} pasos")
    print(f"Δpasos → base={mover1}, hombro={mover2}, codo={mover3}")

    # Direcciones
    dir_base.value   = 1 if mover1 > 0 else 0
    dir_hombro.value = 1 if mover2 > 0 else 0
    dir_codo.value   = 1 if mover3 > 0 else 0

    # --------------------------------------------------
    # PASO 1: despejar ligeramente el codo
    # --------------------------------------------------

    pasos_despeje = int((62.0 / 360.0) * PPV_CODO)

    print("Despejando codo...")

    dir_codo.value = 0

    for _ in range(pasos_despeje):
        step_codo.on()
        time.sleep(RETARDO_PULSO)
        step_codo.off()
        time.sleep(RETARDO_PULSO)


    # --------------------------------------------------
    # PASO 2: mover hombro
    # --------------------------------------------------

    print("Moviendo hombro...")

    dir_hombro.value = 1 if mover2 > 0 else 0

    for _ in range(abs(mover2)):
        step_hombro.on()
        time.sleep(RETARDO_PULSO)
        step_hombro.off()
        time.sleep(RETARDO_PULSO)

    # --------------------------------------------------
    # PASO 3: mover codo al objetivo final
    # --------------------------------------------------

    print("Moviendo codo a objetivo...")

    dir_codo.value = 1 if mover3 > 0 else 0

    for _ in range(abs(mover3)):
        step_codo.on()
        time.sleep(RETARDO_PULSO)
        step_codo.off()
        time.sleep(RETARDO_PULSO)

    # --------------------------------------------------
    # PASO 4: mover base
    # --------------------------------------------------

    print("Moviendo base...")

    dir_base.value = 1 if mover1 > 0 else 0

    for _ in range(abs(mover1)):
        step_base.on()
        time.sleep(RETARDO_PULSO)
        step_base.off()
        time.sleep(RETARDO_PULSO)

    pasos_actuales[0] = p1
    pasos_actuales[1] = p2
    pasos_actuales[2] = p3
    print("Movimiento completado")
    print("Tomando foto final...")

    time.sleep(1)

    global timestamp_actual

    imagen_final = picam2.capture_array()

    cv2.imwrite(
      f"final_{timestamp_actual}.jpg",
      imagen_final
)

    print(f"Foto final guardada: final_{timestamp_actual}.jpg")

def volver_home():
    print("Volviendo a HOME...")
    m1 = -pasos_actuales[0]
    m2 = -pasos_actuales[1]
    m3 = -pasos_actuales[2]

    dir_base.value   = 1 if m1 > 0 else 0
    dir_hombro.value = 1 if m2 > 0 else 0
    dir_codo.value   = 1 if m3 > 0 else 0

    max_pasos = max(abs(m1), abs(m2), abs(m3))
    if max_pasos == 0:
        return

    contador1 = contador2 = contador3 = 0

    for i in range(1, max_pasos + 1):
        if abs(m1) * i // max_pasos > contador1:
            step_base.on()
            contador1 += 1
        if abs(m2) * i // max_pasos > contador2:
            step_hombro.on()
            contador2 += 1
        if abs(m3) * i // max_pasos > contador3:
            step_codo.on()
            contador3 += 1

        time.sleep(RETARDO_PULSO)
        step_base.off()
        step_hombro.off()
        step_codo.off()
        time.sleep(RETARDO_PULSO)

    pasos_actuales[0] = 0
    pasos_actuales[1] = 0
    pasos_actuales[2] = 0
    print("En HOME")

# ── Main ──────────────────────────────────────────────────────────
timestamp_actual = ""
print("Sistema iniciado. Brazo en HOME.")
print("Capturando imagen...")

picam2 = Picamera2()
config = picam2.create_still_configuration(
    main={"size": (4608, 2592), "format": "RGB888"}
)
picam2.configure(config)
picam2.set_controls({"AfMode": 2, "AfTrigger": 0})
picam2.start()
time.sleep(3)
frame = picam2.capture_array()

mascara_roi = np.zeros(frame.shape[:2], dtype=np.uint8)
cv2.fillPoly(mascara_roi, [esquinas], 255)
gris = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
gris = cv2.bitwise_and(gris, gris, mask=mascara_roi)

_, mask = cv2.threshold(gris, 180, 255, cv2.THRESH_BINARY)
kernel = np.ones((5,5), np.uint8)
mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

contornos, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

frame_display = cv2.resize(cv2.cvtColor(frame, cv2.COLOR_RGB2BGR),
                           (int(4608*0.25), int(2592*0.25)))
esquinas_d = (esquinas * 0.25).astype(np.int32)
cv2.polylines(frame_display, [esquinas_d], True, (0, 255, 255), 2)

candidatos = []
for contorno in contornos:
    area = cv2.contourArea(contorno)
    if area < 20000:
        continue
    perimetro = cv2.arcLength(contorno, True)
    approx = cv2.approxPolyDP(contorno, 0.04 * perimetro, True)
    if 4 <= len(approx) <= 6:
        x, y, w, h = cv2.boundingRect(approx)
        if 0.5 < float(w)/h < 2.0:
            candidatos.append((area, contorno, approx, x, y, w, h))

if not candidatos:
    print("No se detectó ningún objeto")
    step_base.off(); step_hombro.off(); step_codo.off()
    exit()

candidatos.sort(key=lambda c: c[0], reverse=True)
area, contorno, approx, x, y, w, h = candidatos[0]
cx = x + w // 2
cy = y + h // 2

mm = pixel_a_mm(cx, cy)
mx, my = mm[0], mm[1]

xb, yb, zb = camara_a_brazo(mx, my)
print(f"Objeto en cámara:  ({mx:.1f}, {my:.1f}) mm")
print(f"Coordenadas brazo: ({xb:.1f}, {yb:.1f}, {zb:.1f}) mm")

resultado = cinematica_inversa(xb, yb, zb)

if not resultado:
    print("Punto fuera de alcance o límites físicos")
    step_base.off(); step_hombro.off(); step_codo.off()
    exit()
t1, t2, t3 = resultado
t2 -= 10 
t3 += 18
print(f"Ángulos → θ1={t1:.1f}°  θ2={t2:.1f}°  θ3={t3:.1f}°")

cx_d     = int(cx * 0.25)
cy_d     = int(cy * 0.25)
x_d      = int(x  * 0.25)
y_d      = int(y  * 0.25)
approx_d = (approx * 0.25).astype(np.int32)
cv2.drawContours(frame_display, [approx_d], -1, (0, 255, 0), 3)
cv2.circle(frame_display, (cx_d, cy_d), 8, (0, 0, 255), -1)
cv2.putText(frame_display,
           f"({mx:.0f}mm,{my:.0f}mm) t1={t1:.1f} t2={t2:.1f} t3={t3:.1f}",
           (x_d, y_d - 10),
           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
timestamp_actual = datetime.now().strftime("%Y%m%d_%H%M%S")

cv2.imwrite(
    f"detectado_{timestamp_actual}.jpg",
    frame
)

print(f"Imagen guardada: detectado_{timestamp_actual}.jpg")

input("\nPresiona ENTER para mover el brazo al objeto...")
ir_a_angulos(t1, t2, t3)
print("Brazo en posición")

input("Presiona ENTER para volver a HOME...")
volver_home()
picam2.stop()
step_base.off(); step_hombro.off(); step_codo.off()
print("Sistema apagado")
