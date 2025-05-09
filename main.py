from machine import Pin, PWM, I2C, ADC
import time
import neopixel
import vl53l0x
import robot
import machine

# --- NeoPixels on GPIO 14 (6 LED) ---
np = neopixel.NeoPixel(Pin(14), 6)

def set_all(color):
    for i in range(6): np[i] = color
    np.write()

def show_detected(fl=False, fr=False, l=False, r=False):
    set_all((0, 0, 0))  # clear
    if fl: np[2] = (0, 128, 255)
    if fr: np[3] = (0, 128, 255)
    if l:  np[1] = (255, 255, 0)
    if r:  np[4] = (255, 165, 0)
    np.write()

def wait_for_button(pin):
    while pin.value():
        time.sleep_ms(50)
    while not pin.value():
        time.sleep_ms(50)

def calibrate():
    print("🔘 Press WHITE button 5x with sensor on WHITE area")
    set_all((255, 255, 255))
    white_vals = []
    while len(white_vals) < 5:
        wait_for_button(btn_white)
        val = (flrL.read() + flrR.read()) // 2
        print("  white", len(white_vals)+1, "=", val)
        white_vals.append(val)
    white_val = sum(white_vals) // len(white_vals)
    print("✅ WHITE calibrated:", white_val)

    print("🔘 Press BLACK button 5x with sensor on BLACK area")
    set_all((0, 0, 80))  # dark blue
    black_vals = []
    while len(black_vals) < 5:
        wait_for_button(btn_black)
        val = (flrL.read() + flrR.read()) // 2
        print("  black", len(black_vals)+1, "=", val)
        black_vals.append(val)
    black_val = sum(black_vals) // len(black_vals)
    print("✅ BLACK calibrated:", black_val)

    edge_thresh = (white_val + black_val) // 2
    print("⚙️ Edge threshold:", edge_thresh)
    return edge_thresh

# --- IR signal output (38 kHz) ---
PWM(Pin(5), freq=38000, duty_u16=0x8000)

# --- IR sensors ---
senFL = Pin(34, Pin.IN, Pin.PULL_UP)
senFR = Pin(23, Pin.IN, Pin.PULL_UP)
senL  = Pin(35, Pin.IN, Pin.PULL_UP)
senR  = Pin(4,  Pin.IN, Pin.PULL_UP)

# --- Distance sensor ---
i2c = I2C(0, sda=Pin(32), scl=Pin(33))
try:
    tof = vl53l0x.VL53L0X(i2c)
    tof.start()
    tof_ok = True
except:
    print("❗ ToF sensor not found")
    tof_ok = False

# --- Floor sensors ---
flrL = ADC(Pin(36))
flrR = ADC(Pin(39))
flrL.atten(ADC.ATTN_11DB)
flrR.atten(ADC.ATTN_11DB)

# --- Buttons ---
btn_white = Pin(27, Pin.IN, Pin.PULL_UP)
btn_black = Pin(26, Pin.IN, Pin.PULL_UP)

# --- Initialize motors ---
i2c_motors = I2C(1, sda=Pin(21), scl=Pin(22))
rob = robot.Robot(i2c_motors)

# === Calibration ===
edge_thresh = calibrate()

# === Wait for start long press ===
print("🕹️ Hold WHITE button to start fight")
set_all((0, 255, 0))  # Green to show ready
pressed = False
start_time = time.ticks_ms()
while not pressed:
    if not btn_white.value():
        start_time = time.ticks_ms()
        while not btn_white.value():
            if time.ticks_diff(time.ticks_ms(), start_time) > 2000:
                pressed = True
                break
    time.sleep_ms(100)

print("🥊 FIGHT STARTS IN 5 SECONDS")
set_all((255, 255, 255))
time.sleep(5)

# === Main fight loop ===
while True:
    # Restart or halt conditions
    if not btn_white.value():
        print("🔁 RESTARTING FROM BEGINNING")
        machine.soft_reset()

    if not btn_black.value():
        print("🛑 HALT BY BLACK BUTTON")
        set_all((0, 0, 0))
        rob.drive(0, 0)
        while True:
            time.sleep(1)

    fl = senFL.value() == 0
    fr = senFR.value() == 0
    ir_l = senL.value() == 0
    ir_r = senR.value() == 0

    # ToF
    dist = tof.read() if tof_ok else 9999
    close = 30 < dist < 500

    # Floor
    floorL = flrL.read()
    floorR = flrR.read()
    edgeL = floorL > edge_thresh
    edgeR = floorR > edge_thresh

    if edgeL or edgeR:
        print("❗ EDGE DETECTED — SPIN 180°!")
        set_all((255, 0, 0))  # red
        rob.drive(20000, -20000)
        time.sleep(0.8)
        rob.drive(0, 0)
        time.sleep(0.2)
        continue

    # LED + logic
    show_detected(fl, fr, ir_l, ir_r)

    if fl and fr:
        print("🟩 ENEMY AHEAD → DRIVE FORWARD")
        rob.drive(-25000, -25000)
    elif fl and ir_l:
        print("🟦 FRONT-LEFT + LEFT (<50cm) → TURN RIGHT")
        rob.drive(-15000, -25000)
    elif fr and ir_r:
        print("🟧 FRONT-RIGHT + RIGHT (<50cm) → TURN LEFT")
        rob.drive(-25000, -15000)
    elif ir_l and ir_r:
        print("🟨 ENEMY ON SIDES → REORIENT")
        rob.drive(20000, -20000)
    elif fl:
        print("🔵 FRONT-LEFT → TURN RIGHT")
        rob.drive(-12000, -24000)
    elif fr:
        print("🔵 FRONT-RIGHT → TURN LEFT")
        rob.drive(-24000, -12000)
    elif ir_l and close:
        print("🟡 LEFT + CLOSE → SPIN RIGHT")
        rob.drive(15000, -20000)
    elif ir_r and close:
        print("🟠 RIGHT + CLOSE → SPIN LEFT")
        rob.drive(-20000, 15000)
    elif ir_l:
        print("🟡 ONLY LEFT SENSOR → SPIN RIGHT")
        rob.drive(15000, -20000)
    elif ir_r:
        print("🟠 ONLY RIGHT SENSOR → SPIN LEFT")
        rob.drive(-20000, 15000)
    else:
        print("🔄 SEARCHING...")
        rob.drive(15000, -15000)

    print(f"   ToF: {dist} mm | FloorL: {floorL} | FloorR: {floorR} | Threshold: {edge_thresh}")
    time.sleep(0.3)

