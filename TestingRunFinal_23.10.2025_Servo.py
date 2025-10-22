# =========================================
# YOLOv8 + FUZZY + ARDUINO LCD SYSTEM (FINAL - SERVO REALTIME)
# PWM dari FPS+Confidence, Servo dari jarak objek (refleks realtime)
# Diperbaiki: pengiriman serial PWM selalu dikirim saat berubah / minimal interval
# =========================================

from ultralytics import YOLO
import cv2, time, serial, numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl
import csv
from pathlib import Path

# ======================
# 1️⃣ LOAD MODEL YOLO
# ======================
model_path = r"D:\Kuliah Politeknik Negeri Madiun\SEMESTER 7\3. KONTROL CERDAS\Week 9 (UTS)\Dataset Training_20.10.2025_15.45\train2\weights\best.pt"
model = YOLO(model_path)

# ======================
# 2️⃣ INISIALISASI KAMERA DAN ARDUINO
# ======================
camera = cv2.VideoCapture(0)
camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
camera.set(cv2.CAP_PROP_FPS, 15)

try:
    arduino = serial.Serial('COM5', 9600, timeout=1)
    time.sleep(2)
except:
    arduino = None

prev_time = time.time()
prev_pwm_out = 0
pwm_default = 50

# --- pengiriman serial control ---
last_serial_send = 0.0
serial_send_interval = 0.20  # detik minimal antar kirim
last_sent_pwm = None

# ======================
# 3️⃣ FUZZY LOGIC PWM (FPS + Confidence)
# ======================
fps = ctrl.Antecedent(np.arange(0, 16, 1), 'fps')
conf = ctrl.Antecedent(np.arange(0, 1.01, 0.01), 'confidence')
pwm = ctrl.Consequent(np.arange(0, 256, 1), 'pwm')

fps['lambat'] = fuzz.trimf(fps.universe, [0, 0, 7])
fps['cukup']  = fuzz.trimf(fps.universe, [5, 7.5, 10])
fps['ideal']  = fuzz.trimf(fps.universe, [6, 15, 15])

conf['rendah'] = fuzz.trimf(conf.universe, [0, 0, 0.5])
conf['sedang'] = fuzz.trimf(conf.universe, [0.3, 0.55, 0.8])
conf['tinggi'] = fuzz.trimf(conf.universe, [0.5, 1, 1])

pwm['minimal']  = fuzz.trimf(pwm.universe, [0, 0, 80])
pwm['normal']   = fuzz.trimf(pwm.universe, [80, 160, 200])
pwm['maksimal'] = fuzz.trimf(pwm.universe, [200, 255, 255])

rule_pwm = [
    ctrl.Rule(fps['lambat'] & conf['rendah'], pwm['minimal']),
    ctrl.Rule(fps['lambat'] & conf['sedang'], pwm['minimal']),
    ctrl.Rule(fps['lambat'] & conf['tinggi'], pwm['normal']),
    ctrl.Rule(fps['cukup']  & conf['rendah'], pwm['minimal']),
    ctrl.Rule(fps['cukup']  & conf['sedang'], pwm['normal']),
    ctrl.Rule(fps['cukup']  & conf['tinggi'], pwm['maksimal']),
    ctrl.Rule(fps['ideal']  & conf['rendah'], pwm['normal']),
    ctrl.Rule(fps['ideal']  & conf['sedang'], pwm['maksimal']),
    ctrl.Rule(fps['ideal']  & conf['tinggi'], pwm['maksimal'])
]
fuzzy_ctrl_pwm = ctrl.ControlSystem(rule_pwm)
fuzzy_sim_pwm = ctrl.ControlSystemSimulation(fuzzy_ctrl_pwm)

# ======================
# 4️⃣ FUZZY LOGIC SERVO (jarak objek)
# ======================
jarak_input = ctrl.Antecedent(np.arange(0, 401, 1), 'jarak_input')
servo_out = ctrl.Consequent(np.arange(0, 181, 1), 'servo_out')

jarak_input['pendek'] = fuzz.trimf(jarak_input.universe, [0, 0, 100])
jarak_input['sedang'] = fuzz.trimf(jarak_input.universe, [80, 150, 220])
jarak_input['panjang'] = fuzz.trimf(jarak_input.universe, [200, 400, 400])

servo_out['turun'] = fuzz.trimf(servo_out.universe, [0, 0, 90])
servo_out['standby'] = fuzz.trimf(servo_out.universe, [60, 90, 120])
servo_out['naik'] = fuzz.trimf(servo_out.universe, [90, 180, 180])

rule_servo = [
    ctrl.Rule(jarak_input['pendek'], servo_out['turun']),
    ctrl.Rule(jarak_input['sedang'], servo_out['standby']),
    ctrl.Rule(jarak_input['panjang'], servo_out['naik'])
]
fuzzy_ctrl_servo = ctrl.ControlSystem(rule_servo)
fuzzy_sim_servo = ctrl.ControlSystemSimulation(fuzzy_ctrl_servo)

# ======================
# 5️⃣ WARNA BOUNDING BOX
# ======================
colors = {
    "Tutup Botol": (0, 200, 0),
    "Penghapus": (0, 0, 255),
    "Sticky Note": (255, 0, 0)
}

# ======================
# 6️⃣ LIST LOG CSV
# ======================
log_data = []

# ======================
# 7️⃣ LOOP DETEKSI REALTIME
# ======================
last_jarak = 0  # 🧩 [Tambahan Realtime Servo] inisialisasi nilai awal

def send_serial_if_needed(pwm_val, jarak_lbl, servo_lbl):
    """
    Kirim ke Arduino jika:
      - pwm berubah signifikan dari terakhir (>=1), OR
      - servo perlu update, OR
      - minimal interval sejak pengiriman terakhir terlewati
    """
    global last_serial_send, last_sent_pwm
    now = time.time()
    should_send = False

    # kirim jika pwm berbeda >= 1
    if last_sent_pwm is None or abs(int(pwm_val) - int(last_sent_pwm)) >= 1:
        should_send = True

    # juga boleh override dengan interval minimal agar status tetap ter-refresh
    if now - last_serial_send >= serial_send_interval:
        should_send = True

    if should_send and arduino:
        send_data = f"{int(pwm_val)},{jarak_lbl},{servo_lbl}\n"
        try:
            arduino.write(send_data.encode())
            # optional: flush not necessary for pyserial, but short sleep avoids overload
            # time.sleep(0.01)
            last_serial_send = now
            last_sent_pwm = int(pwm_val)
            # debug print agar kamu bisa lihat apa yang dikirim
            print(f"[SERIAL SENT] {send_data.strip()}")
        except Exception as e:
            print("[SERIAL ERROR]", e)

while True:
    ret, frame = camera.read()
    if not ret:
        break

    frame_h, frame_w = frame.shape[:2]
    active_area = (frame_w//4, 0, frame_w, frame_h)

    curr_time = time.time()
    fps_value = 1 / (curr_time - prev_time)
    prev_time = curr_time

    results = model(frame, verbose=False)

    # Ambil objek di area aktif
    active_confs = []
    active_boxes = []
    centers = []
    for box in results[0].boxes:
        x1, y1, x2, y2 = map(int, box.xyxy[0])
        cx = (x1 + x2) // 2
        cy = (y1 + y2) // 2
        if active_area[0] <= cx <= active_area[2] and active_area[1] <= cy <= active_area[3]:
            active_confs.append(float(box.conf[0]))
            active_boxes.append(box)
            centers.append(cx)

    fps_input = np.clip(fps_value, 0, 15)
    conf_input = np.clip(np.median(active_confs) if active_confs else 0, 0, 1)

    log_data.append([time.strftime("%H:%M:%S"), f"{fps_input:.2f}", f"{conf_input:.2f}"])

    # FUZZY PWM
    try:
        if not active_confs:
            pwm_out = pwm_default
            prev_pwm_out = pwm_out
        else:
            fuzzy_sim_pwm.reset()
            fuzzy_sim_pwm.input['fps'] = fps_input
            fuzzy_sim_pwm.input['confidence'] = conf_input
            fuzzy_sim_pwm.compute()
            pwm_out = fuzzy_sim_pwm.output['pwm']
            pwm_out = pwm_out * (fps_input / 15)
            pwm_out = np.clip(pwm_out, 0, 255)
            pwm_out = pwm_out
            prev_pwm_out = pwm_out
    except:
        pwm_out = pwm_default

    # FUZZY SERVO (jarak antar 2 objek pertama)
    if len(centers) >= 2:
        jarak = abs(centers[1] - centers[0])
    else:
        jarak = 200

    try:
        fuzzy_sim_servo.reset()
        fuzzy_sim_servo.input['jarak_input'] = jarak
        fuzzy_sim_servo.compute()
        servo_val = fuzzy_sim_servo.output['servo_out']
    except:
        servo_val = 90  # standby

    # 🧩 [Tambahan Realtime Servo] — deteksi perubahan jarak cepat
    if abs(jarak - last_jarak) > 5:  # threshold perubahan posisi antar objek
        update_servo = True
        last_jarak = jarak
    else:
        update_servo = False

    # Tentukan label jarak & PWM
    jarak_label = "PENDEK" if jarak < 90 else "SEDANG" if jarak < 140 else "PANJANG"
    speed_label = "MINIMAL" if pwm_out < 80 else "NORMAL" if pwm_out < 200 else "MAKSIMAL"
    servo_label = "TURUN" if servo_val < 60 else "STANDBY" if servo_val < 120 else "NAIK"

    # 🧩 [Kirim Serial] : kirim saat perlu (pwm berubah / interval / update_servo)
    # Pastikan selalu ada newline '\n' di akhir string
    send_flag = False
    # kirim kalau servo perlu update (refleks)
    if update_servo:
        send_flag = True

    # atau kirim kalau pwm berubah signifikan / interval kadaluarsa
    # fungsi send_serial_if_needed akan cek lebih lanjut
    if arduino:
        send_serial_if_needed(pwm_out, jarak_label, servo_label)

    # GAMBAR BOUNDING BOX
    for box in active_boxes:
        x1, y1, x2, y2 = map(int, box.xyxy[0])
        class_id = int(box.cls[0])
        class_name = results[0].names[class_id]
        color = colors.get(class_name, (0,0,0))
        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
        label = f"{class_name}: {float(box.conf[0]):.2f}"
        cv2.putText(frame, label, (x1, y1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

    # INFO DISPLAY
    black = (0,0,0)
    cv2.putText(frame, f"FPS: {fps_input:.1f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, black, 2)
    cv2.putText(frame, f"Confidence: {conf_input:.2f}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, black, 2)
    cv2.putText(frame, f"PWM: {pwm_out:.0f} ({speed_label})", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, black, 2)
    cv2.putText(frame, f"Jarak: {jarak:.0f} ({jarak_label}) Servo:{servo_label}", (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.7, black, 2)

    cv2.rectangle(frame, (active_area[0], active_area[1]), (active_area[2], active_area[3]), (0,255,255), 2)
    cv2.imshow("YOLO + FUZZY + ARDUINO SYSTEM", frame)

    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        save_csv = input("Apakah ingin menyimpan log CSV? (y/n): ").strip().lower()
        if save_csv == 'y':
            csv_file = Path("log_fps_conf.csv")
            with csv_file.open("w", newline="") as f:
                writer = csv.writer(f)
                writer.writerow(["Time", "FPS", "Confidence"])
                writer.writerows(log_data)
            print(f"✅ CSV disimpan di {csv_file.resolve()}")
        else:
            print("❌ CSV tidak disimpan.")
        break

camera.release()
cv2.destroyAllWindows()
if arduino:
    arduino.close()
print("🔚 Program selesai.")
