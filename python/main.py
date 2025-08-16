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

app = Flask(__name__)

# Global variables - READ FROM TRACKBARV2 CONFIG
cap = None

# Load HSV values from trackbarv2 config or use defaults
def load_trackbarv2_config():
    """Load configuration from trackbarv2.py color_detection_config.yaml"""
    try:
        with open('color_detection_config.yaml', 'r') as file:
            config = yaml.safe_load(file)
            print("✅ Loaded trackbarv2 configuration from color_detection_config.yaml")
            return config
    except FileNotFoundError:
        print("⚠️  trackbarv2 config file not found, using default HSV values")
        return None
    except Exception as e:
        print(f"❌ Error loading trackbarv2 config: {e}")
        return None

# Initialize values from trackbarv2 config or defaults
trackbarv2_config = load_trackbarv2_config()

if trackbarv2_config and 'hsv' in trackbarv2_config:
    # Use HSV values from trackbarv2
    hsv_values = {
        'h_min': trackbarv2_config['hsv']['h_min'],
        's_min': trackbarv2_config['hsv']['s_min'], 
        'v_min': trackbarv2_config['hsv']['v_min'],
        'h_max': trackbarv2_config['hsv']['h_max'],
        's_max': trackbarv2_config['hsv']['s_max'],
        'v_max': trackbarv2_config['hsv']['v_max']
    }
    print(f"🎨 Using HSV values from trackbarv2: {hsv_values}")
else:
    # Default HSV values for green color detection if config not found
    hsv_values = {
        'h_min': 33, 's_min': 159, 'v_min': 89,
        'h_max': 83, 's_max': 233, 'v_max': 190
    }
    print("🎨 Using default HSV values")

# Load morphology values from trackbarv2 or use defaults
if trackbarv2_config and 'morphology' in trackbarv2_config:
    morphology_values = {
        'erosion': trackbarv2_config['morphology']['erosion'],
        'dilation': trackbarv2_config['morphology']['dilation'],
        'opening': trackbarv2_config['morphology']['opening'],
        'closing': trackbarv2_config['morphology']['closing']
    }
    print(f"🔧 Using morphology values from trackbarv2: {morphology_values}")
else:
    # Default morphology values
    morphology_values = {
        'erosion': 1, 'dilation': 3, 'opening': 2, 'closing': 1
    }
    print("🔧 Using default morphology values")

# Load calibration values from trackbarv2 or use defaults
if trackbarv2_config and 'calibration' in trackbarv2_config:
    calibration_values = {
        'calib_distance_cm': trackbarv2_config['calibration']['calib_distance_cm'],
        'calib_pixel_area': trackbarv2_config['calibration']['calib_pixel_area'],
        'min_area_threshold': trackbarv2_config['calibration']['min_area_threshold']
    }
    print(f"📏 Using calibration values from trackbarv2: {calibration_values}")
else:
    # Default calibration values
    calibration_values = {
        'calib_distance_cm': 50,
        'calib_pixel_area': 5000,
        'min_area_threshold': 500
    }
    print("📏 Using default calibration values")

# Load camera controls from trackbarv2 or use defaults
if trackbarv2_config and 'camera_controls' in trackbarv2_config:
    camera_controls_from_trackbar = {
        'auto_exposure': trackbarv2_config['camera_controls']['auto_exposure'],
        'exposure': trackbarv2_config['camera_controls']['exposure'],
        'auto_wb': trackbarv2_config['camera_controls']['auto_wb'],
        'wb_temperature': trackbarv2_config['camera_controls']['wb_temperature'],
        'brightness': trackbarv2_config['camera_controls']['brightness'],
        'contrast': trackbarv2_config['camera_controls']['contrast']
    }
    print(f"📷 Using camera controls from trackbarv2: {camera_controls_from_trackbar}")
else:
    camera_controls_from_trackbar = {
        'auto_exposure': 0, 'exposure': 50, 'auto_wb': 0,
        'wb_temperature': 40, 'brightness': 50, 'contrast': 50
    }
    print("📷 Using default camera controls")

# NEW: Control mode and PID parameters
control_mode = 'manual'  # 'manual' or 'auto'
# FIXED: Track last key pressed from web interface
last_key_pressed = 'none'
pid_params = {
    'setpoint_jarak': 50.0,
    'kp_jarak': 1.0, 'ki_jarak': 0.1, 'kd_jarak': 0.05,
    'kp_arahhadap': 1.0, 'ki_arahhadap': 0.1, 'kd_arahhadap': 0.05
}

camera_controls = camera_controls_from_trackbar.copy()  # Use values from trackbarv2
object_data = {
    'x': 0, 'y': 0, 'distance': 0,
    'error_x': 0, 'error_y': 0, 'area': 0
}
camera_lock = Lock()
serial_port = None

# Initialize serial connection
def initialize_serial():
    global serial_port
    try:
        serial_port = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
        print("✅ Serial connection established on /dev/ttyUSB0")
    except Exception as e:
        print(f"❌ Serial connection failed: {e}")
        serial_port = None

def send_serial_data():
    global serial_port, object_data, control_mode, last_key_pressed, pid_params
    if serial_port and serial_port.is_open:
        try:
            # Use the last key pressed from web interface
            tombol_ditekan = last_key_pressed
            
            # Create data string - EXACT FORMAT AS REQUESTED
            data = f"{{{object_data['error_x']},{object_data['error_y']},{object_data['distance']},{object_data['area']},{control_mode},{tombol_ditekan},{pid_params['setpoint_jarak']},{pid_params['kp_jarak']},{pid_params['ki_jarak']},{pid_params['kd_jarak']},{pid_params['kp_arahhadap']},{pid_params['ki_arahhadap']},{pid_params['kd_arahhadap']}}}\n"
            
            serial_port.write(data.encode())
            print(f"📤 Serial TX: {data.strip()}")
            
        except Exception as e:
            print(f"❌ Serial send error: {e}")

# Load initial config (prioritize control_config.yaml, then trackbarv2 config)
def load_config():
    global control_mode, pid_params, camera_controls
    try:
        # Try to load control_config.yaml first (for PID and control mode)
        with open('control_config.yaml', 'r') as file:
            config = yaml.safe_load(file)
            control_mode = config.get('control_mode', 'manual')
            pid_params = config.get('pid_params', pid_params)
            
            # Use camera controls from control_config if available, otherwise use trackbarv2 values
            if 'camera_controls' in config:
                camera_controls.update(config['camera_controls'])
            else:
                # Keep camera_controls from trackbarv2
                pass
                
            print(f"✅ Control config loaded - Mode: {control_mode}")
            print(f"📷 Camera controls: {camera_controls}")
    except FileNotFoundError:
        print("⚠️  control_config.yaml not found, using trackbarv2 camera values and default control settings")

def save_config():
    config = {
        'control_mode': control_mode,
        'pid_params': pid_params,
        'camera_controls': camera_controls,
        # Also save the loaded HSV, morphology and calibration values for backup
        'hsv_values': hsv_values,
        'morphology_values': morphology_values,
        'calibration_values': calibration_values
    }
    
    with open('control_config.yaml', 'w') as file:
        yaml.dump(config, file, default_flow_style=False)
    print(f"✅ Config saved - Control mode: {control_mode}")
    print(f"💾 Saved HSV values: {hsv_values}")
    print(f"💾 Saved morphology values: {morphology_values}")
    print(f"💾 Saved calibration values: {calibration_values}")

def initialize_camera():
    global cap
    if cap is None:
        print("🎥 Trying to initialize camera...")
        camera_indices = [0]
        
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
        
        # Get frame dimensions
        frame_height, frame_width = frame.shape[:2]
        frame_center_x, frame_center_y = frame_width // 2, frame_height // 2
        
        # Convert BGR to HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # Create HSV range (FIXED VALUES)
        lower = np.array([hsv_values['h_min'], hsv_values['s_min'], hsv_values['v_min']])
        upper = np.array([hsv_values['h_max'], hsv_values['s_max'], hsv_values['v_max']])
        
        # Create mask
        mask = cv2.inRange(hsv, lower, upper)
        
        # Apply fixed morphology operations
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
        
        # Apply mask to original frame
        result = cv2.bitwise_and(frame, frame, mask=mask)
        
        # Convert mask to 3 channel for display
        mask_3ch = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        
        # Create frame with bounding box and tracking info
        frame_with_box = frame.copy()
        
        # Add center crosshair (yellow)
        cv2.line(frame_with_box, (0, frame_center_y), (frame_width, frame_center_y), (0, 255, 255), 1)
        cv2.line(frame_with_box, (frame_center_x, 0), (frame_center_x, frame_height), (0, 255, 255), 1)
        cv2.circle(frame_with_box, (frame_center_x, frame_center_y), 3, (0, 255, 255), -1)
        
        # Find contours and draw bounding box
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if contours:
            # Find largest contour
            largest_contour = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest_contour)
            
            # Only process if area is above threshold
            if area > calibration_values['min_area_threshold']:
                x, y, w, h = cv2.boundingRect(largest_contour)
                
                # Calculate distance based on calibration
                current_pixel_area = area
                
                if calibration_values['calib_pixel_area'] > 0 and calibration_values['calib_distance_cm'] > 0:
                    estimated_distance = calibration_values['calib_distance_cm'] * (calibration_values['calib_pixel_area'] / current_pixel_area) ** 0.5
                else:
                    estimated_distance = 0
                
                # Draw bounding box (green)
                cv2.rectangle(frame_with_box, (x, y), (x + w, y + h), (0, 255, 0), 2)
                
                # Object center
                center_x, center_y = x + w//2, y + h//2
                
                # Calculate error from frame center
                error_x = center_x - frame_center_x
                error_y = center_y - frame_center_y
                
                # Draw object center crosshair (red)
                cv2.circle(frame_with_box, (center_x, center_y), 5, (0, 0, 255), -1)
                cv2.line(frame_with_box, (center_x-10, center_y), (center_x+10, center_y), (0, 0, 255), 2)
                cv2.line(frame_with_box, (center_x, center_y-10), (center_x, center_y+10), (0, 0, 255), 2)
                
                # Draw line from center to object (cyan)
                cv2.line(frame_with_box, (frame_center_x, frame_center_y), (center_x, center_y), (255, 255, 0), 2)
                
                # Add text information
                cv2.putText(frame_with_box, f'Area: {int(area)} px', (x, y - 30), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                cv2.putText(frame_with_box, f'Distance: {estimated_distance:.1f} cm', (x, y - 10), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                cv2.putText(frame_with_box, f'Error X: {error_x:+d} px', (x, y - 50), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)
                cv2.putText(frame_with_box, f'Error Y: {error_y:+d} px', (x, y - 70), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)
                
                # Update global object data
                object_data.update({
                    'x': center_x, 'y': center_y, 'distance': estimated_distance,
                    'error_x': error_x, 'error_y': error_y, 'area': int(area)
                })
                
            else:
                # Reset object data if no valid object found
                object_data.update({
                    'x': 0, 'y': 0, 'distance': 0,
                    'error_x': 0, 'error_y': 0, 'area': 0
                })
        else:
            # Reset object data if no contours found
            object_data.update({
                'x': 0, 'y': 0, 'distance': 0,
                'error_x': 0, 'error_y': 0, 'area': 0
            })
        
        # Send serial data
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

@app.route('/update_control_mode', methods=['POST'])
def update_control_mode():
    global control_mode, last_key_pressed
    try:
        data = request.get_json()
        control_mode = data['mode']
        # Reset last key when switching modes
        last_key_pressed = 'none'
        print(f"🔄 Control mode changed to: {control_mode}")
        return jsonify({'status': 'success'})
    except Exception as e:
        return jsonify({'status': 'error', 'message': str(e)})

# NEW: Route to update last key pressed from web interface
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
        print("🚀 Starting Object Tracking Control System with TrackbarV2 Integration...")
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
    app.run(debug=False, host='0.0.0.0', port=5000, threaded=True)
