# ==========================================
# FIRMWARE FINAL ROBOT ROS2 - CALIBRADO
# ==========================================
import machine
import utime
import sys
import select
from machine import Timer

# --- 1. CONFIGURACIÓN DE PINES ---
# Motores
IN1 = machine.PWM(machine.Pin(26)); IN1.freq(20000)
IN2 = machine.PWM(machine.Pin(27)); IN2.freq(20000)
IN3 = machine.PWM(machine.Pin(16)); IN3.freq(20000)
IN4 = machine.PWM(machine.Pin(17)); IN4.freq(20000)

# Encoders
pinA_A = machine.Pin(18, machine.Pin.IN, machine.Pin.PULL_UP)
pinA_B = machine.Pin(19, machine.Pin.IN, machine.Pin.PULL_UP)
pinB_A = machine.Pin(20, machine.Pin.IN, machine.Pin.PULL_UP)
pinB_B = machine.Pin(21, machine.Pin.IN, machine.Pin.PULL_UP)

# --- 2. TUS VALORES CALIBRADOS (NO TOCAR) ---
TICKS_PER_METER = 2325 #2860.0   # Calibración de distancia
PID_KP = 10.0              # Calibración de fuerza
PID_KI = 12.0              # Calibración de error acumulado
MIN_POWER = 12000          # Zona muerta para arrancar
CONTROL_FREQ = 100          # 20 Hz (50ms)

# --- 3. VARIABLES GLOBALES ---
enc_L_speed = 0; enc_R_speed = 0
enc_L_abs = 0;   enc_R_abs = 0
target_L_ticks = 0; target_R_ticks = 0

# --- 4. INTERRUPCIONES DE ENCODERS ---
def isr_enc_L(pin):
    global enc_L_speed, enc_L_abs
    val = 1 if pinA_B.value() == 0 else -1
    enc_L_speed += val
    enc_L_abs += val

def isr_enc_R(pin):
    global enc_R_speed, enc_R_abs
    # Ajusta el signo aquí si cuenta al revés
    val = -1 if pinB_B.value() == 0 else 1 
    enc_R_speed += val
    enc_R_abs += val

pinA_A.irq(trigger=machine.Pin.IRQ_RISING, handler=isr_enc_L)
pinB_A.irq(trigger=machine.Pin.IRQ_RISING, handler=isr_enc_R)

# --- 5. CLASE PID ---
class MotorPID:
    def __init__(self, kp, ki, min_pwm, max_pwm=65535):
        self.kp = kp
        self.ki = ki
        self.min_pwm = min_pwm
        self.max_pwm = max_pwm
        self.integral = 0
        self.reset()

    def reset(self):
        self.integral = 0

    def compute(self, target, measured):
        error = target - measured
        self.integral += error
        
        # Anti-Windup
        limit = self.max_pwm / (self.ki if self.ki > 0 else 1)
        self.integral = max(min(self.integral, limit), -limit)

        output = (self.kp * error) + (self.ki * self.integral)

        # Zona muerta
        if output > 0 and output < self.min_pwm: output = self.min_pwm
        if output < 0 and output > -self.min_pwm: output = -self.min_pwm
        
        return int(max(min(output, self.max_pwm), -self.max_pwm))

pid_L = MotorPID(PID_KP, PID_KI, min_pwm=MIN_POWER)
pid_R = MotorPID(PID_KP, PID_KI, min_pwm=MIN_POWER)

# --- 6. CONTROLADOR DE MOTORES (TIMER) ---
def set_motor_raw(pin_fwd, pin_rev, pwm):
    if pwm >= 0:
        pin_fwd.duty_u16(pwm)
        pin_rev.duty_u16(0)
    else:
        pin_fwd.duty_u16(0)
        pin_rev.duty_u16(abs(pwm))

def control_loop(timer):
    global enc_L_speed, enc_R_speed, target_L_ticks, target_R_ticks
    
    # Lectura de velocidad (Ticks/Seg)
    meas_L = enc_L_speed * CONTROL_FREQ
    meas_R = enc_R_speed * CONTROL_FREQ
    enc_L_speed = 0
    enc_R_speed = 0
    
    # Cálculo PID
    out_L = pid_L.compute(target_L_ticks, meas_L)
    out_R = pid_R.compute(target_R_ticks, meas_R)
    
    # Aplicar
    set_motor_raw(IN1, IN2, out_L)
    set_motor_raw(IN3, IN4, out_R)

tim = Timer(-1)
tim.init(freq=CONTROL_FREQ, mode=Timer.PERIODIC, callback=control_loop)

# --- 7. BUCLE PRINCIPAL (COMUNICACIÓN ROS) ---
print("ROS2 READY. Calibrado K=10/10")

last_sent_L_abs = 0
last_sent_R_abs = 0
last_odom_send = utime.ticks_ms()

while True:
    # Leer comandos (m/s)
    if select.select([sys.stdin], [], [], 0)[0]:
        line = sys.stdin.readline().strip()
        parts = line.split()
        if len(parts) == 3 and parts[0] == "CMD":
            try:
                # Convertir m/s -> Ticks/s usando TU calibración
                target_L_ticks = float(parts[1]) * TICKS_PER_METER
                target_R_ticks = float(parts[2]) * TICKS_PER_METER
            except: pass

    # Enviar Odometría (DELTA de Ticks) a 20 Hz
    now = utime.ticks_ms()
    if utime.ticks_diff(now, last_odom_send) >= 50: # 20Hz
        # calcula deltas desde el último envío
        delta_L = enc_L_abs - last_sent_L_abs
        delta_R = enc_R_abs - last_sent_R_abs

        # protege contra wrap o saltos raros: limita a un rango razonable
        # por ejemplo, ±(TICKS_PER_METER * 5) por ciclo sería absurdo, así que lo recortamos
        max_reasonable = int(TICKS_PER_METER * 10)  # ajusta si tu encoder cuenta muy rápido
        if delta_L > max_reasonable: delta_L = max_reasonable
        if delta_L < -max_reasonable: delta_L = -max_reasonable
        if delta_R > max_reasonable: delta_R = max_reasonable
        if delta_R < -max_reasonable: delta_R = -max_reasonable

        # Enviar DELTAS (no absolutos)
        sys.stdout.write("ENC_D {} {}\n".format(delta_L, delta_R))
        

        # actualizar last sent
        last_sent_L_abs = enc_L_abs
        last_sent_R_abs = enc_R_abs

        last_odom_send = now

    utime.sleep_ms(1)
