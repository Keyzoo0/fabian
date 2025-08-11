from flask import Flask, render_template, Response, jsonify, request
import cv2
import numpy as np
import yaml
import base64
import json
import serial
import time
from threading import Lock

app = Flask(__name__)

# Global variables
cap = None
hsv_values = {
    'h_min': 33, 's_min': 159, 'v_min': 89,
    'h_max': 83, 's_max': 233, 'v_max': 190
}
morphology_values = {
    'erosion': 1, 'dilation': 3, 'opening': 2, 'closing': 1
}
calibration_values = {
    'calib_distance_cm': 50,
    'calib_pixel_area': 5000,  # EXACT SAME AS TRACKBARV2
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
            hsv_values = config.get('hsv', hsv_values)
            morphology_values = config.get('morphology', morphology_values)
            calibration_values = config.get('calibration', calibration_values)
            camera_controls = config.get('camera_controls', camera_controls)
    except FileNotFoundError:
        print("Config file not found, using default values")

def save_config():
    config = {
        'hsv': hsv_values,
        'morphology': morphology_values,
        'calibration': calibration_values,
        'camera_controls': camera_controls
    }
    
    with open('color_detection_config.yaml', 'w') as file:
        yaml.dump(config, file, default_flow_style=False)

def initialize_camera():
    global cap
    if cap is None:
        cap = cv2.VideoCapture(2)  # Try camera index 2 first
        if not cap.isOpened():
            cap = cv2.VideoCapture(0)  # Fallback to default camera

def apply_camera_settings():
    global cap, camera_controls
    if cap is None:
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
        pass  # Some cameras might not support all controls

def process_frame():
    global cap, hsv_values, morphology_values, calibration_values, object_data
    
    with camera_lock:
        if cap is None:
            initialize_camera()
            apply_camera_settings()
        
        ret, frame = cap.read()
        if not ret:
            return None, None, None, None, None
        
        # Get frame dimensions
        frame_height, frame_width = frame.shape[:2]
        frame_center_x, frame_center_y = frame_width // 2, frame_height // 2
        
        # Convert BGR to HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # Create HSV range
        lower = np.array([hsv_values['h_min'], hsv_values['s_min'], hsv_values['v_min']])
        upper = np.array([hsv_values['h_max'], hsv_values['s_max'], hsv_values['v_max']])
        
        # Create mask
        mask = cv2.inRange(hsv, lower, upper)
        
        # Apply morphology operations - SAME AS TRACKBARV2
        if morphology_values['erosion'] > 0:
            kernel = np.ones((morphology_values['erosion'], morphology_values['erosion']), np.uint8)
            mask = cv2.erode(mask, kernel, iterations=1)
        
        if morphology_values['dilation'] > 0:
            kernel = np.ones((morphology_values['dilation'], morphology_values['dilation']), np.uint8)
            mask = cv2.dilate(mask, kernel, iterations=1)
        
        if morphology_values['opening'] > 0:
            kernel = np.ones((morphology_values['opening'], morphology_values['opening']), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        
        if morphology_values['closing'] > 0:
            kernel = np.ones((morphology_values['closing'], morphology_values['closing']), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        
        # Apply mask to original frame
        result = cv2.bitwise_and(frame, frame, mask=mask)
        
        # Convert mask to 3 channel for display
        mask_3ch = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        
        # Create frame with bounding box and tracking info
        frame_with_box = frame.copy()
        
        # Add center crosshair (yellow) - EXACT SAME AS TRACKBARV2
        cv2.line(frame_with_box, (0, frame_center_y), (frame_width, frame_center_y), (255, 255, 0), 1)
        cv2.line(frame_with_box, (frame_center_x, 0), (frame_center_x, frame_height), (255, 255, 0), 1)
        cv2.circle(frame_with_box, (frame_center_x, frame_center_y), 3, (255, 255, 0), -1)
        
        # Find contours and draw bounding box
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if contours:
            # Find largest contour
            largest_contour = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest_contour)
            
            # Only process if area is above threshold
            if area > calibration_values['min_area_threshold']:
                x, y, w, h = cv2.boundingRect(largest_contour)
                
                # Calculate distance based on calibration - EXACT SAME AS TRACKBARV2
                current_pixel_area = area
                if calibration_values['calib_pixel_area'] > 0 and calibration_values['calib_distance_cm'] > 0:
                    estimated_distance = calibration_values['calib_distance_cm'] * (calibration_values['calib_pixel_area'] / current_pixel_area) ** 0.5
                else:
                    estimated_distance = 0  # Belum dikalibrasi  # Belum dikalibrasi
                
                # Draw bounding box (green)
                cv2.rectangle(frame_with_box, (x, y), (x + w, y + h), (0, 255, 0), 2)
                
                # Object center
                center_x, center_y = x + w//2, y + h//2
                
                # Calculate error from frame center - EXACT SAME AS TRACKBARV2
                error_x = center_x - frame_center_x  # Positif = kanan, Negatif = kiri
                error_y = center_y - frame_center_y  # Positif = bawah, Negatif = atas
                
                # Draw object center crosshair (red) - EXACT SAME AS TRACKBARV2
                cv2.circle(frame_with_box, (center_x, center_y), 5, (255, 0, 0), -1)
                cv2.line(frame_with_box, (center_x-10, center_y), (center_x+10, center_y), (255, 0, 0), 2)
                cv2.line(frame_with_box, (center_x, center_y-10), (center_x, center_y+10), (255, 0, 0), 2)
                
                # Draw line from center to object (cyan) - EXACT SAME AS TRACKBARV2
                cv2.line(frame_with_box, (frame_center_x, frame_center_y), (center_x, center_y), (0, 255, 255), 2)
                
                # Add text information - EXACT SAME ORDER AS TRACKBARV2
                cv2.putText(frame_with_box, f'Area: {int(area)} px', (x, y - 30), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                cv2.putText(frame_with_box, f'Distance: {estimated_distance:.1f} cm', (x, y - 10), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
                cv2.putText(frame_with_box, f'Error X: {error_x:+d} px', (x, y - 50), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
                cv2.putText(frame_with_box, f'Error Y: {error_y:+d} px', (x, y - 70), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
                
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
    _, buffer = cv2.imencode('.jpg', frame)
    frame_base64 = base64.b64encode(buffer).decode('utf-8')
    return f"data:image/jpeg;base64,{frame_base64}"

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/get_frames')
def get_frames():
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

@app.route('/update_hsv', methods=['POST'])
def update_hsv():
    global hsv_values
    data = request.get_json()
    hsv_values.update(data)
    return jsonify({'status': 'success'})

@app.route('/update_morphology', methods=['POST'])
def update_morphology():
    global morphology_values
    data = request.get_json()
    morphology_values.update(data)
    return jsonify({'status': 'success'})

@app.route('/update_calibration', methods=['POST'])
def update_calibration():
    global calibration_values
    data = request.get_json()
    calibration_values.update(data)
    return jsonify({'status': 'success'})

@app.route('/update_camera_controls', methods=['POST'])
def update_camera_controls():
    global camera_controls
    data = request.get_json()
    camera_controls.update(data)
    apply_camera_settings()
    return jsonify({'status': 'success'})

@app.route('/get_all_values')
def get_all_values():
    return jsonify({
        'hsv': hsv_values,
        'morphology': morphology_values,
        'calibration': calibration_values,
        'camera_controls': camera_controls,
        'object_data': object_data
    })

@app.route('/save_calibration', methods=['POST'])
def save_calibration():
    try:
        save_config()
        return jsonify({'status': 'success', 'message': 'Configuration saved successfully'})
    except Exception as e:
        return jsonify({'status': 'error', 'message': str(e)})

if __name__ == '__main__':
    load_config()
    initialize_serial()
    app.run(debug=True, host='0.0.0.0', port=5000)