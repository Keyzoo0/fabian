# 🤖 Web-based Object Tracking Control System

A comprehensive web-based control system for object tracking with real-time PID control, designed for differential drive robots. Features HSV color detection, morphological operations, distance calibration, and Arduino-compatible PID controllers.

![System Overview](https://img.shields.io/badge/Python-3.8+-blue) ![Flask](https://img.shields.io/badge/Flask-2.0+-green) ![OpenCV](https://img.shields.io/badge/OpenCV-4.0+-red) ![Arduino](https://img.shields.io/badge/Arduino-Compatible-orange)

## 📋 Table of Contents

- [Features](#-features)
- [System Architecture](#-system-architecture)
- [Installation](#-installation)
- [Hardware Setup](#-hardware-setup)
- [Usage](#-usage)
- [Web Interface](#-web-interface)
- [Configuration](#-configuration)
- [API Reference](#-api-reference)
- [Arduino Integration](#-arduino-integration)
- [Troubleshooting](#-troubleshooting)
- [Contributing](#-contributing)

## 🚀 Features

### 🎯 **Object Tracking**
- Real-time HSV color detection with adjustable parameters
- Morphological operations (erosion, dilation, opening, closing)
- Distance estimation using area-based calibration
- Error calculation (X/Y position relative to frame center)

### 🕹️ **Control Modes**
- **Manual Mode**: Direct robot control using WASD keys
- **Auto Mode**: Autonomous tracking with dual PID controllers

### 📊 **Real-time Monitoring**
- **Error X Chart**: Direction tracking (-20px to +20px tolerance)
- **Distance Chart**: Object distance monitoring (±10cm tolerance)
- **Dual PID Chart**: Distance PID (±600) and Direction PID (±100) outputs
- **20-second sliding window** with Arduino Serial Plotter style visualization

### 🎮 **Robot Simulation**
- Differential drive visualization
- Color-coded wheel states (🟢 Active, 🔴 Inactive)
- Real-time speed indicators
- Direction arrows and movement feedback

### ⚙️ **Web-based Calibration**
- **HSV Tuning**: Hue, Saturation, Value sliders
- **Morphology Controls**: Erosion, dilation, opening, closing
- **Distance Calibration**: Reference distance and pixel area mapping
- **PID Parameter Tuning**: Kp, Ki, Kd for both controllers
- **Live Preview**: Mask and result frames in calibration modal

## 🏗️ System Architecture

```
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   Web Browser   │◄──►│  Flask Server   │◄──►│  Arduino ESP32  │
│  (Interface)    │    │  (main.py)      │    │(serialCommand)  │
└─────────────────┘    └─────────────────┘    └─────────────────┘
         │                       │                       │
         ▼                       ▼                       ▼
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   HTML/CSS/JS   │    │   OpenCV        │    │  Motor Control  │
│   Interface     │    │   Processing    │    │   PID Logic     │
└─────────────────┘    └─────────────────┘    └─────────────────┘
```

### 📁 **Project Structure**
```
📦 python/
├── 📄 main.py                     # Flask web server
├── 📄 README.md                   # This documentation
├── 📄 requirements.txt            # Python dependencies
├── 📄 color_detection_config.yaml # Calibration settings
├── 📄 control_config.yaml         # Control parameters
├── 📁 templates/
│   └── 📄 index.html             # Web interface
├── 📁 templates-backup/           # Original templates
└── 📁 backup/                     # Backup files
```

## 🔧 Installation

### **Prerequisites**
- Python 3.8+
- USB Camera (index 2 by default)
- Arduino ESP32 (optional, for hardware control)
- Serial connection (`/dev/ttyUSB0`)

### **Step 1: Clone Repository**
```bash
git clone <repository-url>
cd python
```

### **Step 2: Install Dependencies**
```bash
pip install -r requirements.txt
```

### **Step 3: Create Requirements File** (if not exists)
```bash
# requirements.txt
Flask==2.3.3
opencv-python==4.8.1.78
PyYAML==6.0.1
pyserial==3.5
numpy==1.24.3
```

### **Step 4: Install Requirements**
```bash
pip install -r requirements.txt
```

## 🔌 Hardware Setup

### **Camera Setup**
1. Connect USB camera to system
2. Verify camera index (default: 2)
3. Adjust camera index in `main.py` if needed:
```python
camera_indices = [2]  # Change if your camera uses different index
```

### **Arduino ESP32 Setup**
1. Upload `serialCommand.ino` to ESP32
2. Connect ESP32 via USB (`/dev/ttyUSB0`)
3. Wire motors according to pin definitions:
```cpp
#define pinM1A 16  // Motor 1A
#define pinM1B 17  // Motor 1B  
#define pinM2A 26  // Motor 2A
#define pinM2B 27  // Motor 2B
```

### **Serial Communication Format**
```
{error_x,error_y,distance,area,mode,key,setpoint,kp_dist,ki_dist,kd_dist,kp_dir,ki_dir,kd_dir}
```

## 🚀 Usage

### **Starting the System**
```bash
cd python
python main.py
```

### **Access Web Interface**
Open browser and navigate to:
```
http://localhost:5000
```

### **Basic Operation**

1. **📹 Start System**: Click "Start System" button
2. **🎮 Choose Mode**: 
   - **Manual**: Use WASD keys for direct control
   - **Auto**: Enable autonomous tracking
3. **🎯 Calibration**: Click "Calibration" to open tuning modal
4. **📊 Monitor**: View real-time charts and robot simulation

## 🖥️ Web Interface

### **Layout Overview**
```
┌─────────────────────────────────────────────────────────────────┐
│ 🎮 Control Panel │        📹 Camera Frames        │ 📊 Charts    │
│                  │  ┌─────────────┬─────────────┐  │             │
│ • System Control │  │   Original  │     HSV     │  │ • Error X   │
│ • Mode Selection │  │             │             │  │ • Distance  │
│ • Manual Keys    │  ├─────────────┼─────────────┤  │ • Dual PID  │
│ • Calibration    │  │    Mask     │   Result    │  │             │
│ • PID Parameters │  │             │             │  │             │
│                  │  └─────────────┴─────────────┘  │             │
│                  │  ┌─────────────────────────────┐  │             │
│                  │  │  🤖 Robot Sim │ 📋 Data     │  │             │
│                  │  └─────────────────────────────┘  │             │
└─────────────────────────────────────────────────────────────────┘
```

### **Control Panel Features**

#### **🎮 System Controls**
- **Start/Stop System**: Main system activation
- **Manual/Auto Mode**: Switch between control modes
- **System Status**: Visual indicator with color coding

#### **⌨️ Manual Controls**
- **W**: Forward (Y=600, Z=0)
- **S**: Backward (Y=-600, Z=0)  
- **A**: Left Turn (Y=0, Z=-100)
- **D**: Right Turn (Y=0, Z=100)
- **None**: Stop (Y=0, Z=0)

#### **⚙️ PID Parameters**
- **Distance PID**: Kp, Ki, Kd for forward/backward control
- **Direction PID**: Kp, Ki, Kd for left/right control
- **Setpoint**: Target distance in centimeters

### **📊 Real-time Charts**

#### **Error X Chart** 🎯
- **Range**: -320px to +320px
- **Tolerance**: ±20px (yellow zone)
- **Purpose**: Object horizontal position tracking

#### **Distance Chart** 📏
- **Range**: 0cm to 200cm
- **Tolerance**: ±10cm around setpoint (yellow zone)
- **Purpose**: Object distance monitoring

#### **Dual PID Chart** 🎮
- **Distance PID**: Blue line (±600 range)
- **Direction PID**: Red line (±100 range)
- **Purpose**: Controller output visualization

### **🎯 Calibration Modal**

#### **HSV Tab** 🌈
```
H Min: [0────●────179]    H Max: [0────●────179]
S Min: [0────●────255]    S Max: [0────●────255]  
V Min: [0────●────255]    V Max: [0────●────255]
```

#### **Morphology Tab** 🔧
```
Erosion:  [0──●──10]    Dilation: [0──●──10]
Opening:  [0──●──10]    Closing:  [0──●──10]
```

#### **Distance Tab** 📏
```
Calibration Distance: [10────●────200] cm
Calibration Area:     [100───●───10000] px
Min Area Threshold:   [50────●────5000] px
```

#### **PID Tab** ⚙️
```
Distance PID:
  Kp: [0.0──●──10.0]  Ki: [0.0──●──2.0]  Kd: [0.0──●──1.0]

Direction PID:  
  Kp: [0.0──●──10.0]  Ki: [0.0──●──2.0]  Kd: [0.0──●──1.0]
  
Setpoint: [10────●────200] cm
```

## ⚙️ Configuration

### **Configuration Files**

#### **color_detection_config.yaml**
```yaml
hsv:
  h_min: 33
  h_max: 83
  s_min: 159
  s_max: 233
  v_min: 89
  v_max: 190

morphology:
  erosion: 1
  dilation: 3
  opening: 2
  closing: 1

calibration:
  calib_distance_cm: 50
  calib_pixel_area: 5000
  min_area_threshold: 500

camera_controls:
  auto_exposure: 0
  exposure: 50
  auto_wb: 0
  wb_temperature: 40
  brightness: 50
  contrast: 50
```

#### **control_config.yaml**
```yaml
control_mode: manual

pid_params:
  setpoint_jarak: 50.0
  kp_jarak: 1.0
  ki_jarak: 0.1
  kd_jarak: 0.05
  kp_arahhadap: 1.0
  ki_arahhadap: 0.1
  kd_arahhadap: 0.05

camera_controls:
  auto_exposure: 0
  exposure: 50
  auto_wb: 0
  wb_temperature: 40
  brightness: 50
  contrast: 50
```

## 📡 API Reference

### **Endpoints**

#### **GET /**
Main web interface

#### **GET /get_frames**
```json
{
  "original": "data:image/jpeg;base64,...",
  "hsv": "data:image/jpeg;base64,...", 
  "mask": "data:image/jpeg;base64,...",
  "result": "data:image/jpeg;base64,..."
}
```

#### **GET /get_calibration_frames**
```json
{
  "mask": "data:image/jpeg;base64,...",
  "result": "data:image/jpeg;base64,..."
}
```

#### **GET /get_calibration_values**
```json
{
  "h_min": 33, "h_max": 83,
  "s_min": 159, "s_max": 233,
  "v_min": 89, "v_max": 190,
  "erosion": 1, "dilation": 3,
  "opening": 2, "closing": 1,
  "calib_distance_cm": 50,
  "calib_pixel_area": 5000,
  "min_area_threshold": 500
}
```

#### **POST /update_calibration_values**
```json
{
  "h_min": 35,
  "s_min": 160,
  "v_min": 90
}
```

#### **POST /save_calibration**
Saves current calibration to YAML file

#### **POST /update_control_mode**
```json
{
  "mode": "auto"
}
```

#### **POST /update_last_key**
```json
{
  "key": "w"
}
```

#### **POST /update_pid_params**
```json
{
  "kp_jarak": 1.5,
  "ki_jarak": 0.2,
  "kd_jarak": 0.1
}
```

#### **GET /get_all_values**
```json
{
  "control_mode": "manual",
  "pid_params": {...},
  "last_key_pressed": "none",
  "camera_controls": {...},
  "object_data": {
    "x": 320, "y": 240,
    "distance": 45.2,
    "error_x": 15, "error_y": -10,
    "area": 4523
  }
}
```

#### **GET /serial_status**
```json
{
  "connected": true,
  "port": "/dev/ttyUSB0"
}
```

## 🤖 Arduino Integration

### **Serial Command Format**
The system sends data to Arduino in this format:
```
{error_x,error_y,distance,area,mode,key,setpoint,kp_dist,ki_dist,kd_dist,kp_dir,ki_dir,kd_dir}
```

### **Example Serial Data**
```
{15,-10,45.2,4523,auto,none,50.0,1.0,0.1,0.05,1.0,0.1,0.05}
```

### **PID Implementation**
The web interface PID calculations exactly match Arduino `serialCommand.ino`:

#### **Distance PID (Forward/Backward)**
- **Input**: Object distance (cm)
- **Setpoint**: Target distance (cm)
- **Output Range**: ±600
- **Tolerance**: ±10cm
- **Sample Time**: 50ms (20Hz)

#### **Direction PID (Left/Right)**
- **Input**: Error X (pixels)
- **Setpoint**: 0 (center)
- **Output Range**: ±100
- **Tolerance**: ±20px
- **Sample Time**: 50ms (20Hz)

### **Arduino PID Parameters**
```cpp
// PID Constraints (matching web interface)
#define MAX_PID_OUTPUT_Y 600   // Distance PID ±600
#define MAX_PID_OUTPUT_Z 100   // Direction PID ±100

// Tolerance Zones
float pid_arah_tolerance = 20.0;   // ±20 pixels
float pid_jarak_tolerance = 10.0;  // ±10 cm

// Sample Time
const unsigned long SAMPLE_TIME = 50;  // 50ms = 20Hz
```

## 🐛 Troubleshooting

### **Common Issues**

#### **Camera Not Found**
```
❌ ERROR: No working camera found!
```
**Solutions:**
1. Check camera connection
2. Verify camera index in code
3. Test with different camera indices: `[0, 1, 2]`
4. Check camera permissions

#### **Serial Connection Failed**
```
❌ Serial connection failed: [Errno 2] No such file or directory: '/dev/ttyUSB0'
```
**Solutions:**
1. Check Arduino connection
2. Verify port name: `ls /dev/tty*`
3. Check user permissions: `sudo usermod -a -G dialout $USER`
4. Try different ports: `/dev/ttyACM0`, `/dev/ttyUSB1`

#### **Web Interface Not Loading**
```
This site can't be reached
```
**Solutions:**
1. Check Flask server is running
2. Verify port 5000 is available
3. Try `http://127.0.0.1:5000` instead
4. Check firewall settings

#### **Poor Object Detection**
**Solutions:**
1. Adjust HSV values in calibration
2. Improve lighting conditions
3. Use solid color objects
4. Adjust morphology operations
5. Increase min area threshold

#### **PID Oscillation**
**Solutions:**
1. Reduce Kp (proportional gain)
2. Increase Kd (derivative gain)
3. Reduce Ki (integral gain)
4. Check tolerance zones
5. Verify sample timing

### **Debug Mode**
Enable detailed logging:
```python
# In main.py
app.run(debug=True, host='0.0.0.0', port=5000)
```

### **Log Analysis**
Monitor console output for:
```
✅ Camera initialized successfully!
📤 Serial TX: {15,-10,45.2,4523,auto,none,50.0,1.0,0.1,0.05,1.0,0.1,0.05}
🎨 Calibration values updated: {'h_min': 35}
⚙️  PID parameters updated: {'kp_jarak': 1.5}
```

## 🤝 Contributing

### **Development Setup**
```bash
git clone <repository-url>
cd python
pip install -r requirements.txt
python main.py
```

### **Code Structure**
- **Backend**: Flask server (`main.py`)
- **Frontend**: HTML/CSS/JavaScript (`templates/index.html`)
- **Arduino**: ESP32 firmware (`serialCommand.ino`)

### **Adding Features**
1. **New Calibration Parameter**:
   - Add to `color_detection_config.yaml`
   - Update calibration modal HTML
   - Add slider event handler
   - Update backend endpoint

2. **New Chart**:
   - Add Chart.js canvas
   - Create chart initialization
   - Add data update logic
   - Style with CSS

3. **New Control Mode**:
   - Add mode selection button
   - Update control logic
   - Add Arduino handling
   - Test thoroughly

### **Pull Request Guidelines**
1. Test all functionality
2. Update documentation
3. Follow existing code style
4. Add comments for complex logic
5. Verify Arduino compatibility

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## 🙏 Acknowledgments

- **OpenCV** for computer vision capabilities
- **Flask** for web framework
- **Chart.js** for real-time visualization
- **Arduino** community for embedded system support

---

## 📞 Support

For issues and questions:
1. Check [Troubleshooting](#-troubleshooting) section
2. Review [API Reference](#-api-reference)
3. Open GitHub issue with:
   - System information
   - Error logs
   - Steps to reproduce
   - Expected vs actual behavior

---

**Built with ❤️ for robotics enthusiasts and computer vision developers**