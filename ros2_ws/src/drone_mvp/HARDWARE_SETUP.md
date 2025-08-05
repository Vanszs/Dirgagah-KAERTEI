# 🔧 KAERTEI 2025 FAIO - Panduan Konfigurasi Hardware

## 📋 Daftar Konfigurasi yang Perlu Disesuaikan

### 1. **PX4/Pixhawk Connection**
```bash
# Cek port yang tersedia
ls /dev/tty* | grep -E "(USB|ACM)"

# Hasil biasanya:
# /dev/ttyUSB0  <- Pixhawk via FTDI
# /dev/ttyACM0  <- Pixhawk via native USB
```

**Edit di `config/hardware_config.conf`:**
```ini
[flight_controller]
connection_port = "/dev/ttyUSB0"    # Ganti sesuai hasil pengecekan
baud_rate = 57600                   # 57600 atau 115200
```

### 2. **Camera Configuration**
```bash
# Cek kamera yang tersedia
v4l2-ctl --list-devices

# Test kamera
cheese /dev/video0  # Front camera
cheese /dev/video2  # Back camera  
cheese /dev/video4  # Top camera
```

**Edit di `config/hardware_config.conf`:**
```ini
[cameras]
front_camera_index = 0    # Sesuai /dev/video0
back_camera_index = 2     # Sesuai /dev/video2
top_camera_index = 4      # Sesuai /dev/video4

# Resolusi (sesuaikan dengan kemampuan kamera)
camera_width = 640
camera_height = 480
camera_fps = 30

# Offset mounting (jika kamera tidak center)
front_camera_offset_x = 0    # Positive = shift right
front_camera_offset_y = 0    # Positive = shift down
```

### 3. **GPIO Pin Configuration (Raspberry Pi)**

**Pinout Reference:** https://pinout.xyz/

```
Recommended Pin Assignment:
┌─────────────────────────────────┐
│  GPIO 18 (Pin 12) → Front Magnet  │
│  GPIO 19 (Pin 35) → Back Magnet   │  
│  GPIO 21 (Pin 40) → Status LED    │
│  GPIO 20 (Pin 38) → Error LED     │
│  GPIO 16 (Pin 36) → Emergency Btn │
│                                 │
│  Ground Pins: 6, 9, 14, 20, 25, │
│               30, 34, 39         │
│  5V Pins: 2, 4                  │
│  3.3V Pins: 1, 17               │
└─────────────────────────────────┘
```

**Edit di `config/hardware_config.conf`:**
```ini
[gpio_pins]
front_magnet_pin = 18       # GPIO untuk relay magnet depan
back_magnet_pin = 19        # GPIO untuk relay magnet belakang
status_led_pin = 21         # GPIO untuk LED status (opsional)
error_led_pin = 20          # GPIO untuk LED error (opsional)
emergency_stop_pin = 16     # GPIO untuk tombol emergency (opsional)

# Relay logic (sesuaikan dengan jenis relay)
relay_active_high = true    # false jika relay aktif dengan LOW
```

### 4. **I2C/ToF Sensor Configuration**
```bash
# Enable I2C
sudo raspi-config
# Interface Options → I2C → Yes

# Cek I2C devices
i2cdetect -y 1

# Output akan seperti:
#      0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
# 00:          -- -- -- -- -- -- -- -- -- -- -- -- -- 
# 10: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
# 20: -- -- -- -- -- -- -- -- -- 29 -- -- -- -- -- -- 
# 30: 30 31 32 -- -- -- -- -- -- -- -- -- -- -- -- --
```

**Edit di `config/hardware_config.conf`:**
```ini
[sensors]
# I2C addresses untuk ToF sensors (jika pakai multiple)
tof_left_address = "0x30"      # Address setelah remap
tof_right_address = "0x31"     # Address setelah remap
tof_front_address = "0x32"     # Address setelah remap

# Default VL53L0X address adalah 0x29
# Jika pakai 1 sensor saja, biarkan default

tof_max_range = 4.0             # Maksimal jarak deteksi (meter)
tof_detection_threshold = 1.5   # Jarak untuk obstacle detection
i2c_bus = 1                     # I2C bus (biasanya 1)
```

### 5. **GPS Waypoints**
**⚠️ PENTING: Update koordinat sesuai lokasi kompetisi!**

```ini
[waypoints]
# Outdoor pickup location (WAJIB DISESUAIKAN!)
outdoor_pickup_latitude = -6.365000    # Latitude lokasi pickup
outdoor_pickup_longitude = 106.825000  # Longitude lokasi pickup
outdoor_pickup_altitude = 30.0         # Altitude relatif (meter)

# Outdoor drop location (WAJIB DISESUAIKAN!)
outdoor_drop_latitude = -6.364500      # Latitude lokasi drop
outdoor_drop_longitude = 106.825500    # Longitude lokasi drop
outdoor_drop_altitude = 30.0           # Altitude relatif (meter)
```

### 6. **Flight Parameters**
```ini
[flight]
# Altitude settings (sesuai peraturan kompetisi)
takeoff_altitude = 1.5          # Altitude takeoff
indoor_cruise_altitude = 1.5    # Max altitude indoor (≤2m aturan KAERTEI)
outdoor_cruise_altitude = 3.0   # Altitude outdoor  
landing_descent_rate = 0.3      # Kecepatan landing (m/s)

# Velocity limits (safety)
max_horizontal_velocity = 2.0   # Max speed horizontal
max_vertical_velocity = 1.0     # Max speed vertical
max_yaw_rate = 45.0            # Max rotation speed (deg/s)

# Safety limits
max_altitude_limit = 10.0       # Hard limit altitude
geofence_radius = 100.0         # Max distance dari home
battery_failsafe_voltage = 14.4 # Low battery voltage (sesuai baterai)
```

### 7. **Computer Vision Settings**
```ini
[vision]
# Detection parameters
detection_confidence_threshold = 0.5    # Confidence threshold
alignment_tolerance_pixels = 30         # Toleransi alignment (pixel)

# Color detection (HSV values) - sesuaikan dengan objek kompetisi
# Red objects
red_lower_hue = 0
red_upper_hue = 10
red_lower_saturation = 50
red_upper_saturation = 255
red_lower_value = 50
red_upper_value = 255

# Blue objects (alternative)
blue_lower_hue = 100
blue_upper_hue = 130
# ... dst

min_object_area = 1000          # Minimum size objek (pixel²)
```

## 🛠️ Cara Setup Step-by-Step

### Step 1: Hardware Testing
```bash
# Jalankan hardware setup script
./hardware_setup.sh

# Pilih option 1: Hardware Detection & Setup
# Script akan otomatis detect semua hardware
```

### Step 2: Edit Konfigurasi
```bash
# Edit file konfigurasi
nano config/hardware_config.conf

# Sesuaikan:
# - Port PX4 sesuai hasil detection
# - Camera index sesuai yang tersedia
# - GPIO pins sesuai wiring
# - GPS coordinates sesuai lokasi kompetisi
```

### Step 3: Validasi Konfigurasi
```bash
# Test konfigurasi
python3 -c "
from drone_mvp.hardware_config import HardwareConfig
config = HardwareConfig()
config.print_summary()
issues = config.validate_config()
if issues:
    for issue in issues: print(issue)
else:
    print('✅ Configuration OK')
"
```

### Step 4: Test Individual Components
```bash
# Jalankan hardware setup script
./hardware_setup.sh

# Pilih option 2: Test Individual Components
# Test satu per satu:
# - PX4 connection
# - Cameras  
# - GPIO/Magnets
# - I2C/ToF sensors
```

## 🔍 Troubleshooting Common Issues

### Issue 1: Permission Denied
```bash
# Add user to required groups
sudo usermod -a -G dialout,video,i2c,gpio $USER

# Logout dan login ulang
# Atau restart system
```

### Issue 2: PX4 Not Connected
```bash
# Cek port tersedia
ls /dev/tty* | grep -E "(USB|ACM)"

# Set permission
sudo chmod 666 /dev/ttyUSB0

# Test dengan QGroundControl
qgroundcontrol
```

### Issue 3: Camera Not Working
```bash
# Cek device tersedia
v4l2-ctl --list-devices

# Test kamera
cheese /dev/video0

# Cek permission
sudo chmod 666 /dev/video*
```

### Issue 4: GPIO Not Working
```bash
# Cek apakah running di Raspberry Pi
cat /proc/cpuinfo | grep "Raspberry Pi"

# Enable GPIO
sudo raspi-config
# Interface Options → SPI → Yes
# Interface Options → I2C → Yes

# Install GPIO tools
sudo apt install gpiod
```

### Issue 5: I2C Not Working  
```bash
# Enable I2C
sudo raspi-config
# Interface Options → I2C → Yes

# Load I2C modules
sudo modprobe i2c-dev
sudo modprobe i2c-bcm2708

# Add to boot
echo 'i2c-dev' | sudo tee -a /etc/modules
echo 'i2c-bcm2708' | sudo tee -a /etc/modules

# Reboot
sudo reboot
```

## 📝 Quick Setup Checklist

- [ ] **PX4 Connection**: Port detected dan bisa konek via QGroundControl
- [ ] **Cameras**: Semua 3 kamera terdeteksi dan bisa capture
- [ ] **GPIO**: Pin assignment sesuai wiring, magnet bisa on/off
- [ ] **I2C**: ToF sensors terdeteksi di I2C bus
- [ ] **Permissions**: User masuk grup dialout, video, i2c, gpio
- [ ] **GPS Coordinates**: Update sesuai lokasi kompetisi AKTUAL
- [ ] **Flight Limits**: Sesuai peraturan kompetisi (indoor ≤2m, outdoor ≤50m)
- [ ] **Object Colors**: HSV range sesuai objek kompetisi
- [ ] **Network**: WiFi/ethernet untuk remote monitoring (opsional)

## 🚀 Final Validation

Sebelum kompetisi, pastikan:
```bash
# Full system test
./hardware_setup.sh
# Pilih option 3: Full System Test

# Checkpoint mission test
./start_checkpoint_mission.sh
# Test minimal sampai checkpoint TAKEOFF
```

**🎯 Tips untuk Kompetisi:**
1. **Backup konfigurasi** yang sudah working
2. **Test di lokasi** sebelum hari kompetisi
3. **Siapkan spare parts** (kabel USB, kamera, relay)
4. **Print wiring diagram** untuk reference cepat
5. **Catat koordinat GPS** lokasi kompetisi dengan akurat
