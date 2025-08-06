import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import re
from collections import deque
from matplotlib.ticker import FuncFormatter

# === CONFIGURE SERIAL PORT ===
SERIAL_PORT = 'COM5'      # Change to match your setup
BAUD_RATE = 115200
MAX_POINTS = 100          # Number of data points to show in plot

# === REGEX PATTERN TO EXTRACT ALTITUDES FROM CSV LINE ===
# rel_altitude is field 18, kalman_altitude is field 19 (zero-based)
# === REGEX PATTERN TO EXTRACT ALTITUDES FROM CSV LINE ===
# rel_altitude is field 18, kalman_altitude is field 24 (zero-based)
pattern = re.compile(
    r'(?:[^,]*,){17}(-?\d+(?:\.\d+)?)(?:[^,]*,){6}(-?\d+(?:\.\d+)?)'
)
# === Data buffers ===
raw_alt = deque(maxlen=MAX_POINTS)
filtered_alt = deque(maxlen=MAX_POINTS)

# === Setup serial connection ===
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)

# === Plotting function ===

def update(frame):
    line = ser.readline().decode('utf-8', errors='ignore').strip()
    if line.startswith('[MQTT DEBUG]'):
        line = line[len('[MQTT DEBUG]'):].strip()
    print("Line:", line)
    print("Fields:", len(line.split(',')))
    match = pattern.match(line)
    if match:
        try:
            raw_val = float(match.group(1).strip())
            filtered_val = float(match.group(2).strip())
            raw_alt.append(raw_val)
            filtered_alt.append(filtered_val)

            ax.clear()
            ax.plot(raw_alt, label='Raw Altitude (rel_altitude)', color='blue', marker='o', markersize=2)
            ax.plot(filtered_alt, label='Filtered Altitude (Kalman)', color='green', linestyle='--')

            ax.set_title("Raw vs Filtered Altitude")
            ax.set_xlabel("Time step")
            ax.set_ylabel("Altitude (m)")
            ax.legend()
            ax.grid(True)
            ax.yaxis.set_major_formatter(FuncFormatter(lambda x, _: f'{x:.6f}'))
        except ValueError:
            pass
# === Setup plot ===
fig, ax = plt.subplots()
ani = animation.FuncAnimation(fig, update, interval=200)
plt.tight_layout()
plt.show()