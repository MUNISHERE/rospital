import time
import odrive
import csv
import matplotlib.pyplot as plt
from odrive.enums import AXIS_STATE_CLOSED_LOOP_CONTROL, CONTROL_MODE_VELOCITY_CONTROL


# =============================
# 🔍 KẾT NỐI ODRIVE
# =============================
print("🔍 Finding ODrive...")
dev0 = odrive.find_any(timeout=15)
print(f"✅ Connected: {dev0.serial_number}")

axis = dev0.axis0  # đổi sang axis1 nếu cần

# =============================
# 🧹 RESET LỖI & CHUẨN BỊ
# =============================
print("🧹 Clearing errors...")
dev0.clear_errors()
time.sleep(0.5)

# Cấu hình an toàn
print("⚙️ Applying safe test config...")
axis.motor.config.current_lim = 3.0
axis.motor.config.requested_current_range = 5.0
axis.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
axis.controller.config.vel_gain = 3.5
axis.controller.config.vel_integrator_gain = 0.95
axis.controller.config.vel_ramp_rate = 0.5

# Vào vòng kín
axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
time.sleep(1)

# =============================
# ⚙️ THIẾT LẬP THỬ NGHIỆM
# =============================
test_vel = 3.0        # setpoint turns/s
duration = 30.0       # thời gian chạy (s)
dt = 0.05             # thời gian lấy mẫu

time_data = []
vel_data = []
sp_data = []

# =============================
# 🧾 GHI LOG CSV
# =============================
fname = "odrive_pid_log.csv"
csvfile = open(fname, "w", newline="")
writer = csv.writer(csvfile)
writer.writerow(["t", "setpoint", "vel_fb", "vbus_voltage",
                 "axis_error", "motor_error", "encoder_error"])

print(f"🚀 Start test: setpoint = {test_vel}")
axis.controller.input_vel = test_vel
t0 = time.time()

# =============================
# 🔄 VÒNG LẶP TEST
# =============================
while time.time() - t0 < duration:
    t = time.time() - t0
    vel_fb = axis.encoder.vel_estimate
    vbus = dev0.vbus_voltage
    a_err = axis.error
    m_err = axis.motor.error
    e_err = axis.encoder.error

    time_data.append(t)
    vel_data.append(vel_fb)
    sp_data.append(axis.controller.input_vel)

    writer.writerow([f"{t:.3f}", axis.controller.input_vel, f"{vel_fb:.6f}",
                     f"{vbus:.3f}", f"0x{a_err:X}", f"0x{m_err:X}", f"0x{e_err:X}"])

    if a_err != 0 or m_err != 0 or e_err != 0:
        print(f"⚠️ ERROR detected! axis_err=0x{a_err:X} motor_err=0x{m_err:X} encoder_err=0x{e_err:X}")
        break

    if int(t * 10) % int(1.0 / dt) == 0:
        print(f"t={t:.2f}s set={axis.controller.input_vel:.1f} fb={vel_fb:.3f} vbus={vbus:.2f}V")

    time.sleep(dt)

# Dừng động cơ
axis.controller.input_vel = 0
csvfile.close()

print(f"✅ Test finished. Log saved to {fname}")


# =============================
# 📈 VẼ ĐỒ THỊ
# =============================
plt.figure(figsize=(8, 4))
plt.plot(time_data, vel_data, label="Velocity feedback", linewidth=2)
plt.plot(time_data, sp_data, "--", label="Setpoint", linewidth=2)
plt.xlabel("Time [s]")
plt.ylabel("Velocity [turns/s]")
plt.title("ODrive Velocity PID Response")
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.show()
