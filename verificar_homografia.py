import cv2
import numpy as np
from picamera2 import Picamera2
import time

# Cargar homografía
H = np.load("homografia.npy")
print("Homografia cargada")

esquinas = np.array([
    [460,  1332],
    [2572, 20],
    [3180, 28],
    [4600, 860],
    [4604, 1572],
    [3680, 2580],
    [1324, 2580]
], dtype=np.int32)

def pixel_a_mm(px, py):
    punto = np.float32([[px, py]])
    resultado = cv2.perspectiveTransform(punto.reshape(1, 1, 2), H)
    return resultado[0][0]

# Capturar frame
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
picam2.stop()

# Crear máscara ROI
mascara_roi = np.zeros(frame.shape[:2], dtype=np.uint8)
cv2.fillPoly(mascara_roi, [esquinas], 255)

# Detección solo dentro del área de trabajo
gris = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
gris = cv2.bitwise_and(gris, gris, mask=mascara_roi)
cv2.imwrite("debug_gris.jpg", gris)

_, mask = cv2.threshold(gris, 180, 255, cv2.THRESH_BINARY)
# agrega esto justo abajo:
cv2.imwrite("debug_gris.jpg", mask)
kernel = np.ones((5,5), np.uint8)
mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

contornos, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

frame_display = cv2.resize(cv2.cvtColor(frame, cv2.COLOR_RGB2BGR),
                           (int(4608*0.25), int(2592*0.25)))

# Dibujar área de trabajo en display
esquinas_d = (esquinas * 0.25).astype(np.int32)
cv2.polylines(frame_display, [esquinas_d], True, (255, 255, 0), 2)

# Filtrar candidatos
candidatos = []
for contorno in contornos:
    area = cv2.contourArea(contorno)
    if area < 20000:
        continue
    perimetro = cv2.arcLength(contorno, True)
    approx = cv2.approxPolyDP(contorno, 0.04 * perimetro, True)
    if 4 <= len(approx) <= 6:
        x, y, w, h = cv2.boundingRect(approx)
        ratio = float(w) / h
        if 0.5 < ratio < 2.0:
            candidatos.append((area, contorno, approx, x, y, w, h))

if candidatos:
    candidatos.sort(key=lambda c: c[0], reverse=True)
    area, contorno, approx, x, y, w, h = candidatos[0]
    cx = x + w // 2
    cy = y + h // 2

    mm = pixel_a_mm(cx, cy)
    mx, my = mm[0], mm[1]

    print(f"Icopor detectado:")
    print(f"  Píxeles:    ({cx}, {cy})")
    print(f"  Milímetros: ({mx:.1f} mm, {my:.1f} mm)")

    cx_d = int(cx * 0.25)
    cy_d = int(cy * 0.25)
    x_d  = int(x  * 0.25)
    y_d  = int(y  * 0.25)
    approx_d = (approx * 0.25).astype(np.int32)

    cv2.drawContours(frame_display, [approx_d], -1, (0, 255, 0), 3)
    cv2.circle(frame_display, (cx_d, cy_d), 8, (0, 0, 255), -1)
    cv2.putText(frame_display,
               f"({mx:.0f}mm, {my:.0f}mm)",
               (x_d, y_d - 10),
               cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
else:
    print("No se detecto ningun objeto")

cv2.imwrite("verificacion.jpg", frame_display)
print("Imagen guardada como verificacion.jpg")
