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

# Global variables - FIXED: Same default values as trackbarv2.py
cap = None
hsv_values = {
    'h_min': 0, 's_min': 0, 'v_min': 0,
    'h_max': 179, 's_max': 255, 'v_max': 255  # SAME AS TRACKBARV2
}
morphology_values = {
    'erosion': 0, 'dilation': 0, 'opening': 0, 'closing': 0  # SAME AS TRACKBARV2
}
calibration_values = {
    'calib_distance_cm': 50,
    'calib_pixel_area': 5000,
    'min_area_threshold': 500
}
camera_controls = {
    'auto_exposure': 0, 'exposure': 50, 'auto_wb': 0,
    'wb_temperature': 40, 'brightness': 50, 'contrast': 50
}
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
        print("Serial connection established on /dev/ttyUSB0")
    except Exception as e:
        print(f"Serial connection failed: {e}")
        serial_port = None

def send_serial_data(error_x, error_y, distance):
    global serial_port
    if serial_port and serial_port.is_open:
        try:
            data = f"X:{error_x:+d},Y:{error_y:+d},D:{distance:.1f}\n"
            serial_port.write(data.encode())
        except Exception as e:
            print(f"Serial send error: {e}")

# Load initial config
def load_config():
    global hsv_values, morphology_values, calibration_values, camera_controls
    try:
        with open('color_detection_config.yaml', 'r') as file:
            config = yaml.safe_load(file)
            # FIXED: Use get() with proper defaults that match trackbarv2
            hsv_values = config.get('hsv', {
                'h_min': 0, 's_min': 0, 'v_min': 0,
                'h_max': 179, 's_max': 255, 'v_max': 255
            })
            morphology_values = config.get('morphology', {
                'erosion': 0, 'dilation': 0, 'opening': 0, 'closing': 0
            })
            calibration_values = config.get('calibration', calibration_values)
            camera_controls = config.get('camera_controls', camera_controls)
            print(f"Config loaded - HSV values: {hsv_values}")
    except FileNotFoundError:
        print("Config file not found, using default values")
        print(f"Default HSV values: {hsv_values}")

def save_config():
    config = {
        'hsv': hsv_values,
        'morphology': morphology_values,
        'calibration': calibration_values,
        'camera_controls': camera_controls
    }
    
    with open('color_detection_config.yaml', 'w') as file:
        yaml.dump(config, file, default_flow_style=False)
    print(f"Config saved - HSV values: {hsv_values}")

def initialize_camera():
    global cap
    if cap is None:
        print("Trying to initialize camera...")
        camera_indices = [2, 0, 1, 3, 4, 5]  # Start with index 2, then try others
        
        for camera_index in camera_indices:
            try:
                print(f"Attempting camera index {camera_index}")
                
                backends = [
                    cv2.CAP_V4L2,    # Video4Linux2 (Linux)
                    cv2.CAP_DSHOW,   # DirectShow (Windows)
                    cv2.CAP_ANY      # Any available backend
                ]
                
                for backend in backends:
                    try:
                        test_cap = cv2.VideoCapture(camera_index, backend)
                        print(f"  Trying backend: {backend}")
                        
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
                                print(f"✓ Successfully initialized camera on index {camera_index} with backend {backend}")
                                print(f"  Frame shape: {frame.shape}")
                                return True
                            else:
                                print(f"  Camera opened but failed to read frame")
                                test_cap.release()
                        else:
                            print(f"  Failed to open camera with backend {backend}")
                            test_cap.release()
                    except Exception as e:
                        print(f"  Error with backend {backend}: {e}")
                        try:
                            test_cap.release()
                        except:
                            pass
                        continue
                        
            except Exception as e:
                print(f"Error with camera index {camera_index}: {e}")
                continue
        
        print("❌ ERROR: No working camera found!")
        return False
    return True

def apply_camera_settings():
    global cap, camera_controls
    if cap is None or not cap.isOpened():
        return
    
    try:
        # Auto Exposure
        if camera_controls['auto_exposure'] == 0:
            cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)  # Manual mode
            exposure_mapped = -13 + (camera_controls['exposure'] / 100.0) * 12
            cap.set(cv2.CAP_PROP_EXPOSURE, exposure_mapped)
        else:
            cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.75)  # Auto mode
        
        # White Balance
        cap.set(cv2.CAP_PROP_AUTO_WB, camera_controls['auto_wb'])
        if camera_controls['auto_wb'] == 0:  # Manual WB
            wb_temp_mapped = 2000 + (camera_controls['wb_temperature'] / 80.0) * 6000
            cap.set(cv2.CAP_PROP_WB_TEMPERATURE, wb_temp_mapped)
        
        # Brightness and Contrast
        cap.set(cv2.CAP_PROP_BRIGHTNESS, camera_controls['brightness'])
        cap.set(cv2.CAP_PROP_CONTRAST, camera_controls['contrast'])
        
    except Exception as e:
        print(f"Error applying camera settings: {e}")

def process_frame():
    global cap, hsv_values, morphology_values, calibration_values, object_data
    
    with camera_lock:
        if cap is None or not cap.isOpened():
            print("Camera not initialized, attempting to initialize...")
            if not initialize_camera():
                print("Failed to initialize camera")
                return None, None, None, None, None
            apply_camera_settings()
        
        for attempt in range(5):
            try:
                ret, frame = cap.read()
                if ret and frame is not None and frame.size > 0:
                    if frame.shape[0] > 0 and frame.shape[1] > 0:
                        break
                
                print(f"Frame read attempt {attempt + 1} failed - ret: {ret}, frame valid: {frame is not None}")
                
                if attempt >= 2:
                    print("Multiple frame read failures, attempting camera reinitialization...")
                    if cap:
                        cap.release()
                    cap = None
                    if not initialize_camera():
                        print("Camera reinitialization failed")
                        return None, None, None, None, None
                    apply_camera_settings()
                
                time.sleep(0.1)
                
            except Exception as e:
                print(f"Exception during frame read attempt {attempt + 1}: {e}")
                time.sleep(0.1)
        
        if not ret or frame is None or frame.size == 0:
            print("ERROR: Failed to read valid frame from camera after all attempts")
            if cap:
                cap.release()
            cap = None
            return None, None, None, None, None
        
        # Get frame dimensions
        frame_height, frame_width = frame.shape[:2]
        frame_center_x, frame_center_y = frame_width // 2, frame_height // 2
        
        # Convert BGR to HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # FIXED: Create HSV range - EXACTLY SAME AS TRACKBARV2
        # Debug print untuk troubleshooting
        if hsv_values['h_min'] == 0 and hsv_values['h_max'] == 179 and hsv_values['s_min'] == 0 and hsv_values['s_max'] == 255 and hsv_values['v_min'] == 0 and hsv_values['v_max'] == 255:
            print("HSV range is FULL (should show all white mask for calibration)")
        
        lower = np.array([hsv_values['h_min'], hsv_values['s_min'], hsv_values['v_min']])
        upper = np.array([hsv_values['h_max'], hsv_values['s_max'], hsv_values['v_max']])
        
        # Create mask - EXACTLY SAME AS TRACKBARV2
        mask = cv2.inRange(hsv, lower, upper)
        
        # Apply morphology operations - EXACTLY SAME AS TRACKBARV2
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
        
        # Add center crosshair (yellow) - EXACT SAME AS TRACKBARV2
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
                
                # Calculate distance based on calibration - EXACTLY SAME AS TRACKBARV2
                current_pixel_area = area
                
                # This is the EXACT SAME formula as trackbarv2.py
                if calibration_values['calib_pixel_area'] > 0 and calibration_values['calib_distance_cm'] > 0:
                    estimated_distance = calibration_values['calib_distance_cm'] * (calibration_values['calib_pixel_area'] / current_pixel_area) ** 0.5
                else:
                    estimated_distance = 0  # Belum dikalibrasi
                
                # Draw bounding box (green)
                cv2.rectangle(frame_with_box, (x, y), (x + w, y + h), (0, 255, 0), 2)
                
                # Object center
                center_x, center_y = x + w//2, y + h//2
                
                # Calculate error from frame center - EXACT SAME AS TRACKBARV2
                error_x = center_x - frame_center_x  # Positif = kanan, Negatif = kiri
                error_y = center_y - frame_center_y  # Positif = bawah, Negatif = atas
                
                # Draw object center crosshair (red) - EXACT SAME AS TRACKBARV2
                cv2.circle(frame_with_box, (center_x, center_y), 5, (0, 0, 255), -1)
                cv2.line(frame_with_box, (center_x-10, center_y), (center_x+10, center_y), (0, 0, 255), 2)
                cv2.line(frame_with_box, (center_x, center_y-10), (center_x, center_y+10), (0, 0, 255), 2)
                
                # Draw line from center to object (cyan) - EXACT SAME AS TRACKBARV2
                cv2.line(frame_with_box, (frame_center_x, frame_center_y), (center_x, center_y), (255, 255, 0), 2)
                  
                # Update global object data
                object_data.update({
                    'x': center_x, 'y': center_y, 'distance': estimated_distance,
                    'error_x': error_x, 'error_y': error_y, 'area': int(area)
                })
                
                # Send data via serial
                send_serial_data(error_x, error_y, estimated_distance)
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
        
        return frame, hsv, mask_3ch, result, frame_with_box

def frame_to_base64(frame):
    if frame is None:
        return None
    
    try:
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 85]  # 85% quality
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
        
        for frame_type, frame_data in frames.items():
            if frame_data is None:
                print(f"Failed to encode {frame_type} frame")
                frames[frame_type] = ""
        
        return jsonify(frames)
    
    except Exception as e:
        print(f"Error in get_frames: {e}")
        return jsonify({'error': f'Frame processing error: {str(e)}'})

@app.route('/update_hsv', methods=['POST'])
def update_hsv():
    global hsv_values
    try:
        data = request.get_json()
        print(f"Received HSV update: {data}")
        hsv_values.update(data)
        print(f"Updated HSV values: {hsv_values}")
        return jsonify({'status': 'success'})
    except Exception as e:
        print(f"Error updating HSV: {e}")
        return jsonify({'status': 'error', 'message': str(e)})

@app.route('/update_morphology', methods=['POST'])
def update_morphology():
    global morphology_values
    try:
        data = request.get_json()
        morphology_values.update(data)
        return jsonify({'status': 'success'})
    except Exception as e:
        return jsonify({'status': 'error', 'message': str(e)})

@app.route('/update_calibration', methods=['POST'])
def update_calibration():
    global calibration_values
    try:
        data = request.get_json()
        calibration_values.update(data)
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
            'hsv': hsv_values,
            'morphology': morphology_values,
            'calibration': calibration_values,
            'camera_controls': camera_controls,
            'object_data': object_data
        })
    except Exception as e:
        return jsonify({'error': str(e)})

@app.route('/save_calibration', methods=['POST'])
def save_calibration():
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

@app.route('/test_camera')
def test_camera():
    """Test endpoint to check if camera is working"""
    try:
        if cap is None:
            return jsonify({'status': 'error', 'message': 'Camera not initialized'})
        
        if not cap.isOpened():
            return jsonify({'status': 'error', 'message': 'Camera not opened'})
        
        ret, frame = cap.read()
        if not ret or frame is None:
            return jsonify({'status': 'error', 'message': 'Failed to read frame'})
        
        return jsonify({
            'status': 'success', 
            'message': 'Camera is working',
            'frame_shape': frame.shape,
            'camera_index': 2,
            'properties': {
                'width': cap.get(cv2.CAP_PROP_FRAME_WIDTH),
                'height': cap.get(cv2.CAP_PROP_FRAME_HEIGHT),
                'fps': cap.get(cv2.CAP_PROP_FPS)
            }
        })
    except Exception as e:
        return jsonify({'status': 'error', 'message': str(e)})

@app.route('/test_frame')
def test_frame():
    """Get a single test frame to verify image transmission"""
    try:
        if cap is None or not cap.isOpened():
            return jsonify({'error': 'Camera not available'})
        
        ret, frame = cap.read()
        if not ret or frame is None:
            return jsonify({'error': 'Failed to read frame'})
        
        frame_b64 = frame_to_base64(frame)
        if frame_b64 is None:
            return jsonify({'error': 'Failed to encode frame'})
        
        return jsonify({
            'status': 'success',
            'frame': frame_b64,
            'shape': frame.shape
        })
    except Exception as e:
        return jsonify({'error': str(e)})

if __name__ == '__main__':
    if os.environ.get('WERKZEUG_RUN_MAIN') or not app.debug:
        load_config()
        initialize_serial()
        print("Starting Color Detection Calibration System...")
        print("Attempting to initialize camera on startup (prioritizing index 2)...")
        print(f"Initial HSV values: {hsv_values}")
        
        if initialize_camera():
            print("✓ Camera initialized successfully!")
            if cap:
                width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
                height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
                fps = cap.get(cv2.CAP_PROP_FPS)
                print(f"Camera resolution: {int(width)}x{int(height)} @ {fps} FPS")
        else:
            print("⚠ Warning: Camera initialization failed, will retry when needed.")
    
    print("Flask server starting on http://0.0.0.0:5000")
    print("Access the application at: http://localhost:5000")
    
    app.run(debug=False, host='0.0.0.0', port=5000, threaded=True)