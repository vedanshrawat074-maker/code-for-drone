import time import serial
from serial.serialutil import SerialException


import cflib.crtp
from cflib.crazyflie import Crazyflie


# ================= CONFIG =================
DRONE_URI = "udp://192.168.43.42" # Your LiteWing / Crazyflie URI
BT_PORT = "COM5"	# <-- REPLACE with your OUTGOING ESP32 BT COM BAUDRATE = 115200
RECONNECT_DELAY = 3.0	# seconds between BT reconnect attempts


# VERY SOFT movement settings
MAX_TILT_SPEED = 0.05	# max |vx| / |vy| (m/s) - tiny Z_TARGET = 0.01	# target hover thrust
Z_RAMP_TIME = 4.0		# seconds to ramp 0 -> Z_TARGET DEADBAND = 0.7	# ignore gyro noise [-0.7..0.7]
WARMUP_SAMPLES = 12	# number of trigger=1 samples before actual flight WARMUP_Z_FACTOR = 0.25		# fraction of Z_TARGET used in warmup
# ==========================================
 
def connect_bt():
"""Connect to ESP32 Bluetooth COM port with auto-retry.""" while True:
try:
print(f"[BT] Trying to open {BT_PORT}...")
ser = serial.Serial(BT_PORT, BAUDRATE, timeout=1) print(f"[BT] Connected to ESP32 on {BT_PORT}") return ser
except SerialException as e:
print(f"[BT] Failed to open {BT_PORT}: {e}")
print(f"[BT] Retrying in {RECONNECT_DELAY} sec...") time.sleep(RECONNECT_DELAY)



def main():
# ----- Connect to Bluetooth ESP32 ----- ser = connect_bt()
time.sleep(1)


# ----- Connect to Crazyflie / LiteWing ----- cflib.crtp.init_drivers()
cf = Crazyflie()


print("[CF] Connecting to drone...") cf.open_link(DRONE_URI) print("[CF] Connected to drone.")
 
time.sleep(1.0)


print("[CF] Unlocking safety with zero setpoint...") cf.commander.send_setpoint(0, 0, 0, 0) time.sleep(0.3)

cf.param.set_value('commander.enHighLevel', '1') print("[CF] High-level API active.")

ser.reset_input_buffer()
print("[MAIN] Control loop started. Press Ctrl+C to stop.")


# ----- State variables ----- last_trigger = "0"
z_thrust = 0.0 last_ramp_time = time.time()
warmup_count = 0 # how many trigger=1 samples we've seen


try:
while True:
try:
# Read ESP32 line
line = ser.readline().decode('utf-8', errors='ignore').strip() if not line:
continue
 
parts = line.split(',') if len(parts) < 3:
print("[BAD] Bad line:", line) continue

clean_x = parts[0] clean_y = parts[1] trigger = parts[2]

# ----- TRIGGER OFF (STOP / IDLE) -----
if trigger == "0":
if last_trigger != "0":
print("[CTRL] Trigger OFF → stopping & resetting ramp/warmup") z_thrust = 0.0
last_ramp_time = time.time() warmup_count = 0
last_trigger = "0"


cf.commander.send_setpoint(0, 0, 0, 0) ser.reset_input_buffer()
continue


# ----- TRIGGER ON -----
if trigger == "1": now = time.time()
 
# First time trigger=1 after 0 if last_trigger == "0":
print("[CTRL] Trigger ON → starting warmup + ultra-smooth ramp") z_thrust = 0.0
last_ramp_time = now warmup_count = 0

# Warmup phase: don’t move, just tiny gentle spin-up warmup_count += 1
warm_z = min(Z_TARGET * WARMUP_Z_FACTOR, 0.08) # absolute tiny cap


if warmup_count <= WARMUP_SAMPLES:
print(f"[WARMUP] sample {warmup_count}/{WARMUP_SAMPLES},
z={warm_z:.2f}")
cf.commander.send_hover_setpoint(0.0, 0.0, 0.0, warm_z)
last_trigger = "1" continue

# After warmup, ramp z_thrust 0 -> Z_TARGET very slowly elapsed = now - last_ramp_time
if elapsed < Z_RAMP_TIME:
z_thrust = (elapsed / Z_RAMP_TIME) * Z_TARGET else:
z_thrust = Z_TARGET


# Parse gyro safely
 
try:
gyroX = float(clean_x) gyroY = float(clean_y)
except ValueError:
print("[PARSE] Float error from:", line) last_trigger = trigger
continue


# Deadband: ignore small shakes
if -DEADBAND < gyroX < DEADBAND:
gyroX = 0.0
if -DEADBAND < gyroY < DEADBAND:
gyroY = 0.0


# Convert tilt → velocity (very gentle) vx = round(gyroX / 10.0, 3)
vy = -round(gyroY / 10.0, 3)


# Hard clamp to super small speeds
vx = max(-MAX_TILT_SPEED, min(MAX_TILT_SPEED, vx)) vy = max(-MAX_TILT_SPEED, min(MAX_TILT_SPEED, vy))

print(f"[CTRL] vx={vx:.3f}, vy={vy:.3f}, z={z_thrust:.2f}") cf.commander.send_hover_setpoint(vx, vy, 0.0, z_thrust)

last_trigger = "1"
 
# ----- Lost Bluetooth? Auto reconnect ----- except (SerialException, OSError) as e:
print(f"[BT] LOST CONNECTION: {e}")
print("[BT] Stopping drone + reconnecting...")


cf.commander.send_setpoint(0, 0, 0, 0)


try:
ser.close() except:
pass


ser = connect_bt() ser.reset_input_buffer()

except KeyboardInterrupt: print("\n[MAIN] Ctrl+C → stopping...")

finally:
print("[MAIN] Sending final stop + closing...") try:
cf.commander.send_setpoint(0, 0, 0, 0) except:
pass try:
 
cf.close_link() except:
pass try:
ser.close() except:
pass
print("[MAIN] Shutdown complete.")




if _name_ == "_main_":
main()
