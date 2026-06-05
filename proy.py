import os
import time
import threading
from smbus2 import SMBus
from gpiozero import OutputDevice, AngularServo, Servo

os.environ["SDL_VIDEODRIVER"] = "dummy"
import pygame

# ── Configuración INA226 ──────────────────────────────────────────
ADDRESS    = 0x40
REG_CALIB  = 0x05
REG_BUS    = 0x02
REG_SHUNT  = 0x01
SHUNT_OHMS = 0.1
CORRIENTE_MAX_A = 3.0

bus_i2c = SMBus(1)
bus_i2c.write_word_data(ADDRESS, REG_CALIB, 0x0400)

def leer_registro(reg):
    data = bus_i2c.read_i2c_block_data(ADDRESS, reg, 2)
    valor = (data[0] << 8) | data[1]
    if valor > 32767:
        valor -= 65536
    return valor

def leer_ina226():
    voltaje   = leer_registro(REG_BUS) * 1.25 / 1000
    corriente = (leer_registro(REG_SHUNT) * 2.5 / 1000000) / SHUNT_OHMS
    potencia  = voltaje * corriente
    return voltaje, corriente, potencia

# ── Hilo de monitoreo ─────────────────────────────────────────────
datos_ina = {"voltaje": 0.0, "corriente": 0.0, "potencia": 0.0}
alerta_activa = False

def monitoreo_ina():
    global alerta_activa
    while True:
        try:
            v, c, p = leer_ina226()
            datos_ina["voltaje"]   = v
            datos_ina["corriente"] = c
            datos_ina["potencia"]  = p

            if c > CORRIENTE_MAX_A:
                if not alerta_activa:
                    print(f"\n⚠  ALERTA: Corriente alta → {c:.3f} A  ⚠")
                    alerta_activa = True
            else:
                alerta_activa = False

        except Exception as e:
            pass
        time.sleep(0.5)

hilo_ina = threading.Thread(target=monitoreo_ina, daemon=True)
hilo_ina.start()

# ── GPIO ──────────────────────────────────────────────────────────
print("Inicializando Sistema de Control del Brazo Robotico...")

step_base   = OutputDevice(17)
dir_base    = OutputDevice(27)
step_hombro = OutputDevice(22)
dir_hombro  = OutputDevice(23)
step_codo   = OutputDevice(24)
dir_codo    = OutputDevice(25)

pinza  = AngularServo(18, min_angle=0, max_angle=180,
                      min_pulse_width=0.0005, max_pulse_width=0.0025)
cuello = Servo(19, min_pulse_width=0.001, max_pulse_width=0.002)
pasos_base   = 0
pasos_hombro = 0
pasos_codo   = 0

angulo_pinza = 15.0
pinza.angle  = angulo_pinza
cuello.value = None

VELOCIDAD_PINZA  = 3.0
VELOCIDAD_CUELLO = 0.3
RETARDO_PULSO    = 0.0001
RAFAGA_PASOS     = 20

# ── Pygame ────────────────────────────────────────────────────────
pygame.init()
pygame.joystick.init()

if pygame.joystick.get_count() == 0:
    print("Error Fatal: Mando USB no detectado.")
    pygame.quit()
    exit()

mando = pygame.joystick.Joystick(0)
mando.init()
print(f"Hardware Listo. Operando con: {mando.get_name()}")
print(f"Alerta de sobrecarga configurada en {CORRIENTE_MAX_A} A")
print("─" * 50)

contador = 0

try:
    while True:
        pygame.event.pump()

        mover_base   = False
        mover_hombro = False
        mover_codo   = False

        # --- MOTORES NEMA ---
        if mando.get_numhats() > 0:
            hat_x, hat_y = mando.get_hat(0)
            if hat_x == 1:
                dir_base.value = 1
                mover_base = True
            elif hat_x == -1:
                dir_base.value = 0
                mover_base = True

            if hat_y == 1:
                dir_hombro.value = 1
                mover_hombro = True
            elif hat_y == -1:
                dir_hombro.value = 0
                mover_hombro = True

        if mando.get_button(4):
            dir_codo.value = 1
            mover_codo = True
        elif mando.get_button(5):
            dir_codo.value = 0
            mover_codo = True

        # --- PINZA ---
        if mando.get_button(3):
            angulo_pinza = min(80.0, angulo_pinza + VELOCIDAD_PINZA)
            pinza.angle  = angulo_pinza
        elif mando.get_button(2):
            angulo_pinza = max(15.0, angulo_pinza - VELOCIDAD_PINZA)
            pinza.angle  = angulo_pinza

        # --- CUELLO ---
        l2 = mando.get_axis(2)
        r2 = mando.get_axis(5)

        if l2 > 0.5:
            cuello.value = -VELOCIDAD_CUELLO
        elif r2 > 0.5:
            cuello.value = VELOCIDAD_CUELLO
        else:
            cuello.value = None

        # --- EJECUCION MOTORES ---
        if mover_base or mover_hombro or mover_codo:
            for _ in range(RAFAGA_PASOS):
                if mover_base:   step_base.on()
                if mover_hombro: step_hombro.on()
                if mover_codo:   step_codo.on()
                time.sleep(RETARDO_PULSO)
                step_base.off()
                step_hombro.off()
                step_codo.off()
                time.sleep(RETARDO_PULSO)
            # Actualizar contadores con signo según dirección
            if mover_base:
                pasos_base   += RAFAGA_PASOS if dir_base.value == 1 else -RAFAGA_PASOS
            if mover_hombro:
                pasos_hombro += RAFAGA_PASOS if dir_hombro.value == 1 else -RAFAGA_PASOS
            if mover_codo:
                pasos_codo   += RAFAGA_PASOS if dir_codo.value == 1 else -RAFAGA_PASOS
        else:
            time.sleep(0.01)
        # --- MOSTRAR DATOS CADA 2 SEGUNDOS ---
        contador += 1
        if contador >= 200:
            print(f"V: {datos_ina['voltaje']:.2f}V  I: {datos_ina['corriente']:.3f}A  | base={pasos_base} hombro={pasos_hombro} codo={pasos_codo}")
            contador = 0

except KeyboardInterrupt:
    print("\nParada de emergencia.")
finally:
    step_base.off()
    step_hombro.off()
    step_codo.off()
    pinza.detach()
    cuello.detach()
    bus_i2c.close()
    pygame.quit()
    print("Sistema asegurado y apagado.")
