# ================= FIRMWARE ROBOT DIFF: CONTROL PI PRECISO ==================
import machine
import utime
import sys
import select
from machine import Timer

# ==========================================
# 1. LIBRERÍA PID (Integrada para simplicidad)
# ==========================================
class MotorPID:
    def __init__(self, kp, ki, min_pwm=0, max_pwm=65535):
        self.kp = kp
        self.ki = ki
        self.min_pwm = min_pwm  # Mínima fuerza para vencer fricción
        self.max_pwm = max_pwm
        self.integral = 0
        self.prev_time = utime.ticks_ms()
        self.reset()

    def reset(self):
        self.integral = 0
        self.prev_time = utime.ticks_ms()

    def compute(self, target, measured):
        now = utime.ticks_ms()
        dt = utime.ticks_diff(now, self.prev_time) / 1000.0 # Delta tiempo en segundos
        if dt <= 0: dt = 0.05 # Seguridad
        self.prev_time = now

        error = target - measured

        # --- Término Integral (La Memoria) ---
        # Solo acumulamos si no estamos saturados para evitar "windup"
        self.integral += error * dt
        
        # Limitar la integral para que no crezca infinito
        # (Esto evita que el robot se quede acelerado si lo trabas)
        limit_int = self.max_pwm / (self.ki if self.ki > 0 else 1)
        if self.integral > limit_int: self.integral = limit_int
        elif self.integral < -limit_int: self.integral = -limit_int

        # --- Cálculo Final ---
        output = (self.kp * error) + (self.ki * self.integral)

        # --- Zona Muerta (Deadzone) ---
        # Ayuda al motor a arrancar si la salida es baja pero no cero
        if output > 0 and output < self.min_pwm: output = self.min_pwm
        if output < 0 and output > -self.min_pwm: output = -self.min_pwm
        
        # Limitar al máximo del hardware
        if output > self.max_pwm: output = self.max_pwm
        if output < -self.max_pwm: output = -self.max_pwm

        return int(output)

# ==========================================
# 2. CONFIGURACIÓN DE HARDWARE
# ==========================================

# --- Motores (Driver L298N o similar) ---
IN1 = machine.PWM(machine.Pin(26)); IN1.freq(20000)
IN2 = machine.PWM(machine.Pin(27)); IN2.freq(20000)
IN3 = machine.PWM(machine.Pin(16)); IN3.freq(20000)
IN4 = machine.PWM(machine.Pin(17)); IN4.freq(20000)

def set_motor_raw(pin_fwd, pin_rev, pwm):
    """ Aplica el PWM directo al hardware """
    if pwm >= 0:
        pin_fwd.duty_u16(pwm)
        pin_rev.duty_u16(0)
    else:
        pin_fwd.duty_u16(0)
        pin_rev.duty_u16(abs(pwm))

# --- Encoders (Interrupciones) ---
pinA_A = machine.Pin(18, machine.Pin.IN, machine.Pin.PULL_UP)
pinA_B = machine.Pin(19, machine.Pin.IN, machine.Pin.PULL_UP)
pinB_A = machine.Pin(20, machine.Pin.IN, machine.Pin.PULL_UP)
pinB_B = machine.Pin(21, machine.Pin.IN, machine.Pin.PULL_UP)

# Variables de conteo (Atómicas)
enc_L_speed = 0  # Ticks por intervalo (velocidad)
enc_R_speed = 0
enc_L_abs = 0    # Posición absoluta para ROS
enc_R_abs = 0

def isr_enc_L(pin):
    global enc_L_speed, enc_L_abs
    val = 1 if pinA_B.value() == 0 else -1
    enc_L_speed += val
    enc_L_abs += val

def isr_enc_R(pin):
    global enc_R_speed, enc_R_abs
    # OJO: Si tu motor derecho cuenta al revés, invierte aquí (1 / -1)
    val = -1 if pinB_B.value() == 0 else 1 
    enc_R_speed += val
    enc_R_abs += val

pinA_A.irq(trigger=machine.Pin.IRQ_RISING, handler=isr_enc_L)
pinB_A.irq(trigger=machine.Pin.IRQ_RISING, handler=isr_enc_R)

# ==========================================
# 3. BUCLE DE CONTROL (TIMER)
# ==========================================
# Aquí ocurre la magia. Esto se ejecuta exactamente a 20Hz.

# --- TUNING DEL PID (AQUÍ TOCAS TÚ) ---
# Kp: Dale fuerza. Empieza en 1.0, sube hasta que oscile.
# Ki: Corrige el error acumulado. Sube si no llega a la velocidad deseada.
# Min_PWM: Valor mínimo (0-65535) para que las ruedas empiecen a girar.
PID_KP = 100.0  
PID_KI = 85.0   
MIN_POWER = 15000 

pid_controller_L = MotorPID(PID_KP, PID_KI, min_pwm=MIN_POWER)
pid_controller_R = MotorPID(PID_KP, PID_KI, min_pwm=MIN_POWER)

target_L = 0 # Ticks por segundo objetivo
target_R = 0
CONTROL_FREQ = 20 # Hz (Veces por segundo)

# Escala: 1.0 m/s desde ROS = ¿Cuántos Ticks/segundo en tus ruedas?
# TIENES QUE AJUSTAR ESTO. Pon el robot en el aire, manda 1.0 y mira cuánto cuenta.
# Valor estimado inicial:
TICKS_PER_METER_SEC = 2860.0

def control_loop(timer):
    global enc_L_speed, enc_R_speed, target_L, target_R
    
    # 1. Calcular Velocidad Real (Ticks por segundo)
    # Como el loop corre a 20Hz, multiplicamos lo contado por 20 para tener Ticks/Seg
    measured_L = enc_L_speed * CONTROL_FREQ
    measured_R = enc_R_speed * CONTROL_FREQ
    
    # Reset de contadores parciales
    enc_L_speed = 0
    enc_R_speed = 0
    
    # 2. Calcular PID
    out_L = pid_controller_L.compute(target_L, measured_L)
    out_R = pid_controller_R.compute(target_R, measured_R)
    
    # 3. Aplicar a Motores
    set_motor_raw(IN1, IN2, out_L)
    set_motor_raw(IN3, IN4, out_R)

# Iniciamos el Timer de Hardware (Precisión absoluta)
tim = Timer(-1)
tim.init(freq=CONTROL_FREQ, mode=Timer.PERIODIC, callback=control_loop)

# ==========================================
# 4. LOOP PRINCIPAL (COMUNICACIÓN)
# ==========================================
# Este loop solo se encarga de hablar con la Raspberry Pi.
# El control de motores ocurre en segundo plano gracias al Timer.

print("PID Ready. Freq={}Hz".format(CONTROL_FREQ))
last_odom_send = utime.ticks_ms()

while True:
    # --- Leer Comandos Serial (Non-blocking) ---
    rlist, _, _ = select.select([sys.stdin], [], [], 0)
    if rlist:
        line = sys.stdin.readline().strip()
        parts = line.split()
        if len(parts) == 3 and parts[0] == "CMD":
            try:
                # ROS manda m/s, convertimos a Ticks/s
                v_l = float(parts[1])
                v_r = float(parts[2])
                target_L = v_l * TICKS_PER_METER_SEC
                target_R = v_r * TICKS_PER_METER_SEC
            except: pass

    # --- Enviar Odometría a ROS ---
    now = utime.ticks_ms()
    if utime.ticks_diff(now, last_odom_send) >= 50: # 20Hz
        sys.stdout.write("ENC {} {}\n".format(enc_L_abs, enc_R_abs))
        last_odom_send = now
        
    utime.sleep_ms(1)