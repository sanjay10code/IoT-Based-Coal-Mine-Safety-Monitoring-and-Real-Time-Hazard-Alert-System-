import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.dates as mdates
from collections import deque
import re
import csv
import datetime
from datetime import datetime as dt

# === CONFIGURATION ===
PORT = 'COM3'               # Your ESP32 serial port
BAUD = 115200
MAX_POINTS = 100            # Number of points on the live graph (sliding window)
LOG_FILE = 'sensor_log.csv' # Output CSV file

# Data buffers for live plotting – store datetime objects for x‑axis
time_data = deque(maxlen=MAX_POINTS)
co_data   = deque(maxlen=MAX_POINTS)
o3_data   = deque(maxlen=MAX_POINTS)
ch4_data  = deque(maxlen=MAX_POINTS)
lpg_data  = deque(maxlen=MAX_POINTS)
co2_data  = deque(maxlen=MAX_POINTS)
aqi_data  = deque(maxlen=MAX_POINTS)
temp_data = deque(maxlen=MAX_POINTS)
hum_data  = deque(maxlen=MAX_POINTS)
pm_data   = deque(maxlen=MAX_POINTS)

# === SETUP PLOT (dark theme) ===
plt.style.use('dark_background')
fig, axes = plt.subplots(3, 1, figsize=(12, 8))
fig.tight_layout(pad=4)

# Gas concentrations subplot
ax1 = axes[0]
ax1.set_title('Gas Concentrations (ppm)', color='white')
line_co,  = ax1.plot([], [], label='CO',  color='#FF6384', linewidth=2)
line_o3,  = ax1.plot([], [], label='O3',  color='#36A2EB', linewidth=2)
line_ch4, = ax1.plot([], [], label='CH4', color='#FFCE56', linewidth=2)
line_lpg, = ax1.plot([], [], label='LPG', color='#4BC0C0', linewidth=2)
ax1.legend(loc='upper left', facecolor='#1a1a2e', labelcolor='white')
ax1.grid(True, linestyle='--', alpha=0.3)
ax1.xaxis.set_major_formatter(mdates.DateFormatter('%H:%M:%S'))
ax1.xaxis.set_major_locator(mdates.AutoDateLocator())

# Air Quality & CO₂ subplot
ax2 = axes[1]
ax2.set_title('Air Quality & CO₂', color='white')
line_aqi,  = ax2.plot([], [], label='AQI (%)', color='#9966FF', linewidth=2)
line_co2,  = ax2.plot([], [], label='CO₂ (ppm)', color='#FF9F40', linewidth=2)
ax2.legend(loc='upper left', facecolor='#1a1a2e', labelcolor='white')
ax2.grid(True, linestyle='--', alpha=0.3)
ax2.xaxis.set_major_formatter(mdates.DateFormatter('%H:%M:%S'))
ax2.xaxis.set_major_locator(mdates.AutoDateLocator())

# Environment subplot
ax3 = axes[2]
ax3.set_title('Environment', color='white')
line_temp, = ax3.plot([], [], label='Temp (°C)', color='#FF6384', linewidth=2)
line_hum,  = ax3.plot([], [], label='Humidity (%)', color='#36A2EB', linewidth=2)
line_pm,   = ax3.plot([], [], label='PM2.5 (µg/m³)', color='#4CAF50', linewidth=2)
ax3.legend(loc='upper left', facecolor='#1a1a2e', labelcolor='white')
ax3.grid(True, linestyle='--', alpha=0.3)
ax3.xaxis.set_major_formatter(mdates.DateFormatter('%H:%M:%S'))
ax3.xaxis.set_major_locator(mdates.AutoDateLocator())

# === OPEN SERIAL PORT ===
try:
    ser = serial.Serial(PORT, BAUD, timeout=1)
    print(f"✅ Connected to {PORT}")
except Exception as e:
    print(f"❌ Could not open {PORT}: {e}")
    exit()

# === CSV LOGGING SETUP ===
csv_file = open(LOG_FILE, mode='w', newline='')
csv_writer = csv.writer(csv_file)
csv_writer.writerow([
    'Timestamp', 'CO (ppm)', 'O3 (ppm)', 'CH4 (ppm)', 'LPG (ppm)',
    'CO2 (ppm)', 'AQI (%)', 'Temperature (°C)', 'Humidity (%)', 'PM2.5 (µg/m³)'
])
csv_file.flush()
print(f"📁 Logging to {LOG_FILE}")

def update_plot(frame):
    try:
        line = ser.readline().decode('utf-8').strip()
        if not line:
            return

        if '=== CALIBRATED READINGS ===' in line:
            # Read the next three lines
            mq_line = ser.readline().decode('utf-8').strip()
            gases_line = ser.readline().decode('utf-8').strip()
            env_line = ser.readline().decode('utf-8').strip()

            # Parse MQ‑135 line
            mq_match = re.search(r'\| AQ: (\d+)% \| CO2: (\d+)ppm', mq_line)
            if not mq_match:
                return
            aqi = float(mq_match.group(1))
            co2 = float(mq_match.group(2))

            # Parse gases line
            gases_match = re.search(r'Gases \(actual\): CO=(\d+)ppm \| O3=([\d.]+)ppm \| CH4=(\d+)ppm \| LPG=(\d+)ppm', gases_line)
            if not gases_match:
                return
            co  = float(gases_match.group(1))
            o3  = float(gases_match.group(2))
            ch4 = float(gases_match.group(3))
            lpg = float(gases_match.group(4))

            # Parse environment line (accepts both μ and µ symbols)
            env_match = re.search(r'Env: Temp=([\d.]+)°C \| Hum=(\d+)% \| PM2\.5=([\d.]+)[μµ]g/m³', env_line)
            if not env_match:
                return
            temp = float(env_match.group(1))
            hum  = float(env_match.group(2))
            pm   = float(env_match.group(3))

            # Get current timestamp (for both x‑axis and CSV)
            now = dt.now()

            # Append to live‑plot buffers
            time_data.append(now)
            co_data.append(co)
            o3_data.append(o3)
            ch4_data.append(ch4)
            lpg_data.append(lpg)
            co2_data.append(co2)
            aqi_data.append(aqi)
            temp_data.append(temp)
            hum_data.append(hum)
            pm_data.append(pm)

            # ** LOG TO CSV **
            csv_writer.writerow([
                now.strftime('%Y-%m-%d %H:%M:%S'), co, o3, ch4, lpg,
                co2, aqi, temp, hum, pm
            ])
            csv_file.flush()

            print(f"✅ Logged: {now.strftime('%H:%M:%S')}")

            # Update plots (x‑data = time_data, y‑data = respective buffers)
            line_co.set_data(time_data, co_data)
            line_o3.set_data(time_data, o3_data)
            line_ch4.set_data(time_data, ch4_data)
            line_lpg.set_data(time_data, lpg_data)

            line_aqi.set_data(time_data, aqi_data)
            line_co2.set_data(time_data, co2_data)

            line_temp.set_data(time_data, temp_data)
            line_hum.set_data(time_data, hum_data)
            line_pm.set_data(time_data, pm_data)

            # Adjust x‑axis limits to show the latest data
            if len(time_data) > 1:
                ax1.set_xlim(min(time_data), max(time_data))
                ax2.set_xlim(min(time_data), max(time_data))
                ax3.set_xlim(min(time_data), max(time_data))

            # Rescale y‑axes
            for ax in axes:
                ax.relim()
                ax.autoscale_view(scalex=False)

    except Exception as e:
        print(f"Error in update: {e}")

ani = animation.FuncAnimation(fig, update_plot, interval=500, cache_frame_data=False)

try:
    plt.show()
except KeyboardInterrupt:
    print("\n🛑 Stopped by user")
finally:
    csv_file.close()
    ser.close()
    print("📁 CSV file closed.")