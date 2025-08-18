from flask import Flask, render_template, Response, jsonify, request
import cv2
import numpy as np
import yaml
import base64
import json
import serial
import time
import os
from threading import Lock
from zeroconf import ServiceInfo, Zeroconf
import socket

app = Flask(__name__)

# Global variables
cap = None

# Load HSV values from config or use defaults
def load_color_detection_config():
    """Load configuration from color_detection_config.yaml"""
    try:
        with open('color_detection_config.yaml', 'r') as file:
            config = yaml.safe_load(file)
            print("✅ Loaded color detection configuration from color_detection_config.yaml")
            return config
    except FileNotFoundError:
        print("⚠️  color_detection_config.yaml file not found, using default HSV values")
        return None
    except Exception as e:
        print(f"❌ Error loading color detection config: {e}")
        return None

# Initialize values from config or defaults
color_config = load_color_detection_config()

# HSV values for color detection
if color_config and 'hsv' in color_config:
    hsv_values = {
        'h_min': color_config['hsv']['h_min'],
        's_min': color_config['hsv']['s_min'], 
        'v_min': color_config['hsv']['v_min'],
        'h_max': color_config['hsv']['h_max'],
        's_max': color_config['hsv']['s_max'],
        'v_max': color_config['hsv']['v_max']
    }
    print(f"🎨 Using HSV values from config: {hsv_values}")
else:
    hsv_values = {
        'h_min': 33, 's_min': 159, 'v_min': 89,
        'h_max': 83, 's_max': 233, 'v_max': 190
    }
    print("🎨 Using default HSV values")

# Morphology values
if color_config and 'morphology' in color_config:
    morphology_values = {
        'erosion': color_config['morphology']['erosion'],
        'dilation': color_config['morphology']['dilation'],
        'opening': color_config['morphology']['opening'],
        'closing': color_config['morphology']['closing']
    }
    print(f"🔧 Using morphology values from config: {morphology_values}")
else:
    morphology_values = {
        'erosion': 1, 'dilation': 3, 'opening': 2, 'closing': 1
    }
    print("🔧 Using default morphology values")

# Calibration values
if color_config and 'calibration' in color_config:
    calibration_values = {
        'calib_distance_cm': color_config['calibration']['calib_distance_cm'],
        'calib_pixel_area': color_config['calibration']['calib_pixel_area'],
        'min_area_threshold': color_config['calibration']['min_area_threshold']
    }
    print(f"📏 Using calibration values from config: {calibration_values}")
else:
    calibration_values = {
        'calib_distance_cm': 50,
        'calib_pixel_area': 5000,
        'min_area_threshold': 500
    }
    print("📏 Using default calibration values")

# Camera controls
if color_config and 'camera_controls' in color_config:
    camera_controls_from_config = {
        'auto_exposure': color_config['camera_controls']['auto_exposure'],
        'exposure': color_config['camera_controls']['exposure'],
        'auto_wb': color_config['camera_controls']['auto_wb'],
        'wb_temperature': color_config['camera_controls']['wb_temperature'],
        'brightness': color_config['camera_controls']['brightness'],
        'contrast': color_config['camera_controls']['contrast']
    }
    print(f"📷 Using camera controls from config: {camera_controls_from_config}")
else:
    camera_controls_from_config = {
        'auto_exposure': 0, 'exposure': 50, 'auto_wb': 0,
        'wb_temperature': 40, 'brightness': 50, 'contrast': 50
    }
    print("📷 Using default camera controls")

# Control mode and PID parameters
control_mode = 'manual'
last_key_pressed = 'none'
pid_params = {
    'setpoint_jarak': 50.0,
    'kp_jarak': 1.0, 'ki_jarak': 0.1, 'kd_jarak': 0.05,
    'kp_arahhadap': 1.0, 'ki_arahhadap': 0.1, 'kd_arahhadap': 0.05
}

camera_controls = camera_controls_from_config.copy()
object_data = {
    'x': 0, 'y': 0, 'distance': 0,
    'error_x': 0, 'error_y': 0, 'area': 0
}
camera_lock = Lock()
serial_port = None
zeroconf = None

# Initialize serial connection
def initialize_serial():
    global serial_port
    try:
        serial_port = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
        print("✅ Serial connection established on /dev/ttyUSB0")
    except Exception as e:
        print(f"❌ Serial connection failed: {e}")
        serial_port = None

def get_local_ip():
    """Get the actual local network IP address"""
    try:
        # Connect to a remote address to determine which local IP to use
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.connect(("1.1.1.1", 80))
            return s.getsockname()[0]
    except Exception:
        try:
            # Fallback method
            hostname = socket.gethostname()
            return socket.gethostbyname(hostname)
        except Exception:
            return "127.0.0.1"

def register_mdns_service():
    global zeroconf
    try:
        zeroconf = Zeroconf()
        
        # Get actual local network IP address
        local_ip = get_local_ip()
        
        # Register the service
        service_info = ServiceInfo(
            "_http._tcp.local.",
            "robot._http._tcp.local.",
            addresses=[socket.inet_aton(local_ip)],
            port=5000,
            properties={
                'description': 'Robot Control System'
            },
            server="robot.local."
        )
        
        zeroconf.register_service(service_info)
        print(f"✅ mDNS service registered: robot.local -> {local_ip}:5000")
        
    except Exception as e:
        import traceback
        print(f"⚠️  mDNS registration failed: {e}")
        print(f"Traceback: {traceback.format_exc()}")

def cleanup_mdns():
    global zeroconf
    if zeroconf:
        try:
            zeroconf.close()
            print("✅ mDNS service unregistered")
        except Exception as e:
            print(f"⚠️  Error closing mDNS: {e}")

def send_serial_data():
    global serial_port, object_data, control_mode, last_key_pressed, pid_params
    if serial_port and serial_port.is_open:
        try:
            tombol_ditekan = last_key_pressed
            
            data = f"{{{object_data['error_x']},{object_data['error_y']},{object_data['distance']},{object_data['area']},{control_mode},{tombol_ditekan},{pid_params['setpoint_jarak']},{pid_params['kp_jarak']},{pid_params['ki_jarak']},{pid_params['kd_jarak']},{pid_params['kp_arahhadap']},{pid_params['ki_arahhadap']},{pid_params['kd_arahhadap']}}}\n"
            
            serial_port.write(data.encode())
            print(f"📤 Serial TX: {data.strip()}")
            
        except Exception as e:
            print(f"❌ Serial send error: {e}")

def load_config():
    global control_mode, pid_params, camera_controls
    try:
        with open('control_config.yaml', 'r') as file:
            config = yaml.safe_load(file)
            control_mode = config.get('control_mode', 'manual')
            pid_params = config.get('pid_params', pid_params)
            
            if 'camera_controls' in config:
                camera_controls.update(config['camera_controls'])
                
            print(f"✅ Control config loaded - Mode: {control_mode}")
            print(f"📷 Camera controls: {camera_controls}")
    except FileNotFoundError:
        print("⚠️  control_config.yaml not found, using default control settings")

def save_config():
    config = {
        'control_mode': control_mode,
        'pid_params': pid_params,
        'camera_controls': camera_controls,
        'hsv_values': hsv_values,
        'morphology_values': morphology_values,
        'calibration_values': calibration_values
    }
    
    with open('control_config.yaml', 'w') as file:
        yaml.dump(config, file, default_flow_style=False)
    print(f"✅ Config saved - Control mode: {control_mode}")

def save_calibration_config():
    """Save calibration values to color_detection_config.yaml"""
    config = {
        'hsv': hsv_values,
        'morphology': morphology_values,
        'calibration': calibration_values,
        'camera_controls': camera_controls_from_config
    }
    
    with open('color_detection_config.yaml', 'w') as file:
        yaml.dump(config, file, default_flow_style=False)
    print(f"✅ Calibration config saved to color_detection_config.yaml")

def initialize_camera():
    global cap
    if cap is None:
        print("🎥 Trying to initialize camera...")
        camera_indices = [2]
        
        for camera_index in camera_indices:
            try:
                print(f"   Attempting camera index {camera_index}")
                
                backends = [
                    cv2.CAP_V4L2,
                    cv2.CAP_DSHOW,
                    cv2.CAP_ANY
                ]
                
                for backend in backends:
                    try:
                        test_cap = cv2.VideoCapture(camera_index, backend)
                        
                        if test_cap.isOpened():
                            test_cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
                            test_cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
                            test_cap.set(cv2.CAP_PROP_FPS, 30)
                            
                            for _ in range(5):
                                ret, frame = test_cap.read()
                                if ret and frame is not None:
                                    break
                                time.sleep(0.1)
                            
                            if ret and frame is not None and frame.size > 0:
                                cap = test_cap
                                print(f"✅ Camera initialized on index {camera_index}")
                                return True
                            else:
                                test_cap.release()
                        else:
                            test_cap.release()
                    except Exception as e:
                        try:
                            test_cap.release()
                        except:
                            pass
                        continue
                        
            except Exception as e:
                continue
        
        print("❌ ERROR: No working camera found!")
        return False
    return True

def apply_camera_settings():
    global cap, camera_controls
    if cap is None or not cap.isOpened():
        return
    
    try:
        if camera_controls['auto_exposure'] == 0:
            cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)
            exposure_mapped = -13 + (camera_controls['exposure'] / 100.0) * 12
            cap.set(cv2.CAP_PROP_EXPOSURE, exposure_mapped)
        else:
            cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.75)
        
        cap.set(cv2.CAP_PROP_AUTO_WB, camera_controls['auto_wb'])
        if camera_controls['auto_wb'] == 0:
            wb_temp_mapped = 2000 + (camera_controls['wb_temperature'] / 80.0) * 6000
            cap.set(cv2.CAP_PROP_WB_TEMPERATURE, wb_temp_mapped)
        
        cap.set(cv2.CAP_PROP_BRIGHTNESS, camera_controls['brightness'])
        cap.set(cv2.CAP_PROP_CONTRAST, camera_controls['contrast'])
        
    except Exception as e:
        print(f"⚠️  Error applying camera settings: {e}")

def process_frame():
    global cap, hsv_values, morphology_values, calibration_values, object_data, control_mode
    
    with camera_lock:
        if cap is None or not cap.isOpened():
            if not initialize_camera():
                return None, None, None, None, None
            apply_camera_settings()
        
        for attempt in range(5):
            try:
                ret, frame = cap.read()
                if ret and frame is not None and frame.size > 0:
                    if frame.shape[0] > 0 and frame.shape[1] > 0:
                        break
                
                if attempt >= 2:
                    if cap:
                        cap.release()
                    cap = None
                    if not initialize_camera():
                        return None, None, None, None, None
                    apply_camera_settings()
                
                time.sleep(0.1)
                
            except Exception as e:
                time.sleep(0.1)
        
        if not ret or frame is None or frame.size == 0:
            if cap:
                cap.release()
            cap = None
            return None, None, None, None, None
        
        frame_height, frame_width = frame.shape[:2]
        frame_center_x, frame_center_y = frame_width // 2, frame_height // 2
        
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        lower = np.array([hsv_values['h_min'], hsv_values['s_min'], hsv_values['v_min']])
        upper = np.array([hsv_values['h_max'], hsv_values['s_max'], hsv_values['v_max']])
        
        mask = cv2.inRange(hsv, lower, upper)
        
        if morphology_values['erosion'] > 0:
            kernel_erosion = np.ones((morphology_values['erosion'], morphology_values['erosion']), np.uint8)
            mask = cv2.erode(mask, kernel_erosion, iterations=1)
        
        if morphology_values['dilation'] > 0:
            kernel_dilation = np.ones((morphology_values['dilation'], morphology_values['dilation']), np.uint8)
            mask = cv2.dilate(mask, kernel_dilation, iterations=1)
        
        if morphology_values['opening'] > 0:
            kernel_opening = np.ones((morphology_values['opening'], morphology_values['opening']), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel_opening)
        
        if morphology_values['closing'] > 0:
            kernel_closing = np.ones((morphology_values['closing'], morphology_values['closing']), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel_closing)
        
        result = cv2.bitwise_and(frame, frame, mask=mask)
        
        mask_3ch = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        
        frame_with_box = frame.copy()
        
        cv2.line(frame_with_box, (0, frame_center_y), (frame_width, frame_center_y), (0, 255, 255), 1)
        cv2.line(frame_with_box, (frame_center_x, 0), (frame_center_x, frame_height), (0, 255, 255), 1)
        cv2.circle(frame_with_box, (frame_center_x, frame_center_y), 3, (0, 255, 255), -1)
        
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest_contour)
            
            if area > calibration_values['min_area_threshold']:
                x, y, w, h = cv2.boundingRect(largest_contour)
                
                current_pixel_area = area
                
                if calibration_values['calib_pixel_area'] > 0 and calibration_values['calib_distance_cm'] > 0:
                    estimated_distance = calibration_values['calib_distance_cm'] * (calibration_values['calib_pixel_area'] / current_pixel_area) ** 0.5
                else:
                    estimated_distance = 0
                
                cv2.rectangle(frame_with_box, (x, y), (x + w, y + h), (0, 255, 0), 2)
                
                center_x, center_y = x + w//2, y + h//2
                
                error_x = center_x - frame_center_x
                error_y = center_y - frame_center_y
                
                cv2.circle(frame_with_box, (center_x, center_y), 5, (0, 0, 255), -1)
                cv2.line(frame_with_box, (center_x-10, center_y), (center_x+10, center_y), (0, 0, 255), 2)
                cv2.line(frame_with_box, (center_x, center_y-10), (center_x, center_y+10), (0, 0, 255), 2)
                
                cv2.line(frame_with_box, (frame_center_x, frame_center_y), (center_x, center_y), (255, 255, 0), 2)
                
                cv2.putText(frame_with_box, f'Area: {int(area)} px', (x, y - 30), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                cv2.putText(frame_with_box, f'Distance: {estimated_distance:.1f} cm', (x, y - 10), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                cv2.putText(frame_with_box, f'Error X: {error_x:+d} px', (x, y - 50), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)
                cv2.putText(frame_with_box, f'Error Y: {error_y:+d} px', (x, y - 70), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)
                
                object_data.update({
                    'x': center_x, 'y': center_y, 'distance': estimated_distance,
                    'error_x': error_x, 'error_y': error_y, 'area': int(area)
                })
                
            else:
                object_data.update({
                    'x': 0, 'y': 0, 'distance': 0,
                    'error_x': 0, 'error_y': 0, 'area': 0
                })
        else:
            object_data.update({
                'x': 0, 'y': 0, 'distance': 0,
                'error_x': 0, 'error_y': 0, 'area': 0
            })
        
        send_serial_data()
        
        return frame, hsv, mask_3ch, result, frame_with_box

def frame_to_base64(frame):
    if frame is None:
        return None
    
    try:
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 85]
        _, buffer = cv2.imencode('.jpg', frame, encode_param)
        frame_base64 = base64.b64encode(buffer).decode('utf-8')
        return f"data:image/jpeg;base64,{frame_base64}"
    except Exception as e:
        print(f"Error encoding frame: {e}")
        return None

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/get_frames')
def get_frames():
    try:
        original, hsv, mask, result, frame_with_box = process_frame()
        
        if original is None:
            return jsonify({'error': 'Camera not available'})
        
        frames = {
            'original': frame_to_base64(frame_with_box),
            'hsv': frame_to_base64(hsv),
            'mask': frame_to_base64(mask),
            'result': frame_to_base64(result)
        }
        
        return jsonify(frames)
    
    except Exception as e:
        print(f"Error in get_frames: {e}")
        return jsonify({'error': f'Frame processing error: {str(e)}'})

@app.route('/get_calibration_frames')
def get_calibration_frames():
    """Get frames specifically for calibration modal"""
    try:
        original, hsv, mask, result, frame_with_box = process_frame()
        
        if original is None:
            return jsonify({'error': 'Camera not available'})
        
        frames = {
            'mask': frame_to_base64(mask),
            'result': frame_to_base64(result)
        }
        
        return jsonify(frames)
    
    except Exception as e:
        print(f"Error in get_calibration_frames: {e}")
        return jsonify({'error': f'Calibration frame processing error: {str(e)}'})

@app.route('/get_calibration_values')
def get_calibration_values():
    """Get current calibration values for the modal sliders"""
    try:
        values = {}
        values.update(hsv_values)
        values.update(morphology_values)
        values.update(calibration_values)
        
        return jsonify(values)
    except Exception as e:
        return jsonify({'error': str(e)})

@app.route('/update_calibration_values', methods=['POST'])
def update_calibration_values():
    """Update calibration values from modal sliders"""
    global hsv_values, morphology_values, calibration_values
    try:
        data = request.get_json()
        
        for key, value in data.items():
            if key in hsv_values:
                hsv_values[key] = value
            elif key in morphology_values:
                morphology_values[key] = value
            elif key in calibration_values:
                calibration_values[key] = value
        
        print(f"🎨 Calibration values updated: {data}")
        return jsonify({'status': 'success'})
    except Exception as e:
        return jsonify({'status': 'error', 'message': str(e)})

@app.route('/save_calibration', methods=['POST'])
def save_calibration():
    """Save calibration settings to YAML file"""
    try:
        save_calibration_config()
        return jsonify({'status': 'success', 'message': 'Calibration saved successfully'})
    except Exception as e:
        return jsonify({'status': 'error', 'message': str(e)})

@app.route('/update_control_mode', methods=['POST'])
def update_control_mode():
    global control_mode, last_key_pressed
    try:
        data = request.get_json()
        control_mode = data['mode']
        last_key_pressed = 'none'
        print(f"🔄 Control mode changed to: {control_mode}")
        return jsonify({'status': 'success'})
    except Exception as e:
        return jsonify({'status': 'error', 'message': str(e)})

@app.route('/update_last_key', methods=['POST'])
def update_last_key():
    global last_key_pressed
    try:
        data = request.get_json()
        last_key_pressed = data['key']
        print(f"⌨️  Last key updated from web: {last_key_pressed}")
        return jsonify({'status': 'success'})
    except Exception as e:
        return jsonify({'status': 'error', 'message': str(e)})

@app.route('/update_pid_params', methods=['POST'])
def update_pid_params():
    global pid_params
    try:
        data = request.get_json()
        pid_params.update(data)
        print(f"⚙️  PID parameters updated: {pid_params}")
        return jsonify({'status': 'success'})
    except Exception as e:
        return jsonify({'status': 'error', 'message': str(e)})

@app.route('/update_camera_controls', methods=['POST'])
def update_camera_controls():
    global camera_controls
    try:
        data = request.get_json()
        camera_controls.update(data)
        apply_camera_settings()
        return jsonify({'status': 'success'})
    except Exception as e:
        return jsonify({'status': 'error', 'message': str(e)})

@app.route('/get_all_values')
def get_all_values():
    try:
        return jsonify({
            'control_mode': control_mode,
            'pid_params': pid_params,
            'last_key_pressed': last_key_pressed,
            'camera_controls': camera_controls,
            'object_data': object_data
        })
    except Exception as e:
        return jsonify({'error': str(e)})

@app.route('/save_config', methods=['POST'])
def save_config_route():
    try:
        save_config()
        return jsonify({'status': 'success', 'message': 'Configuration saved successfully'})
    except Exception as e:
        return jsonify({'status': 'error', 'message': str(e)})

@app.route('/serial_status')
def serial_status():
    global serial_port
    try:
        if serial_port and serial_port.is_open:
            return jsonify({'connected': True, 'port': '/dev/ttyUSB0'})
        else:
            return jsonify({'connected': False})
    except:
        return jsonify({'connected': False})

@app.after_request
def after_request(response):
    response.headers.add('Access-Control-Allow-Origin', '*')
    response.headers.add('Access-Control-Allow-Headers', 'Content-Type,Authorization')
    response.headers.add('Access-Control-Allow-Methods', 'GET,PUT,POST,DELETE,OPTIONS')
    response.headers.add('Cache-Control', 'no-cache, no-store, must-revalidate')
    response.headers.add('Pragma', 'no-cache')
    response.headers.add('Expires', '0')
    return response

if __name__ == '__main__':
    if os.environ.get('WERKZEUG_RUN_MAIN') or not app.debug:
        load_config()
        initialize_serial()
        register_mdns_service()
        print("🚀 Starting Object Tracking Control System with Web Calibration...")
        print(f"🎨 HSV Detection Values: H({hsv_values['h_min']}-{hsv_values['h_max']}) S({hsv_values['s_min']}-{hsv_values['s_max']}) V({hsv_values['v_min']}-{hsv_values['v_max']})")
        print(f"🔧 Morphology: erosion={morphology_values['erosion']}, dilation={morphology_values['dilation']}, opening={morphology_values['opening']}, closing={morphology_values['closing']}")
        print(f"📏 Calibration: {calibration_values['calib_distance_cm']}cm = {calibration_values['calib_pixel_area']}px, min_area={calibration_values['min_area_threshold']}")
        print(f"🎮 Control mode: {control_mode}")
        print(f"⚙️  PID parameters: {pid_params}")
        
        if initialize_camera():
            print("✅ Camera initialized successfully!")
        else:
            print("⚠️  Warning: Camera initialization failed, will retry when needed.")
    
    print("🌐 Flask server starting on http://0.0.0.0:5000")
    print("🌐 Access via: http://robot.local:5000")
    
    try:
        app.run(debug=False, host='0.0.0.0', port=5000, threaded=True)
    finally:
        cleanup_mdns()