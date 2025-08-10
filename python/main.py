#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
import threading
import json
import xml.etree.ElementTree as ET
import os
from flask import Flask, render_template, Response, request, jsonify
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class GreenSquareDetector:
    def __init__(self):
        # ROS initialization
        rospy.init_node('green_square_detector', anonymous=True)
        
        # Publishers
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.detection_pub = rospy.Publisher('/detection_data', String, queue_size=10)
        self.control_pub = rospy.Publisher('/control_data', String, queue_size=10)
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # Camera setup
        self.cap = cv2.VideoCapture(2)  # Change to your camera index
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS, 30)
        
        # XML calibration file path
        self.calibration_file = 'greenCalibrationValue.xml'
        
        # Optimized HSV ranges for GREEN detection
        self.hsv_range = {
            'hue_min': 35, 'hue_max': 85,     # Wide green range
            'sat_min': 40, 'sat_max': 255,    # Accept lower saturation
            'val_min': 30, 'val_max': 255     # Accept darker greens
        }
        
        # Alternative green range for better detection
        self.hsv_range_2 = {
            'enabled': True,
            'hue_min': 25, 'hue_max': 45,     # Yellow-green transition
            'sat_min': 30, 'sat_max': 255,
            'val_min': 20, 'val_max': 200
        }
        
        # Visual servoing parameters
        self.visual_servoing = {
            'kp_gain': 0.4,           # Lower gain for smoother control
            'target_size': 80,        # Smaller target for closer approach
            'focal_length': 600,      # Adjusted for green object
            'real_size': 12.0         # Estimated real size in cm
        }
        
        # Optimized morphology for green objects
        self.morphology = {
            'erosion_iterations': 0,   # No erosion to preserve shape
            'dilation_iterations': 2,  # Fill gaps
            'kernel_size': 3,          # Small kernel
            'min_area': 150           # Lower threshold for smaller objects
        }
        
        # Detection parameters optimized for green
        self.detection_params = {
            'min_aspect_ratio': 0.3,   # Very flexible
            'max_aspect_ratio': 3.0,   # Very flexible
            'min_contour_area': 150,   # Lower threshold
            'max_contour_area': 50000, # Higher threshold
            'contour_approximation': 0.02  # Shape approximation
        }
        
        # Load calibration from XML
        self.load_calibration()
        
        # Control state
        self.current_mode = 'auto'
        self.detection_data = {
            'detected': False,
            'center_x': 0,
            'center_y': 0,
            'area': 0,
            'distance': 0,
            'error_x': 0,
            'confidence': 0
        }
        
        # Manual control state
        self.joystick_data = {'x': 0, 'y': 0}
        self.keyboard_data = {'active_keys': [], 'command': 'STOP'}
        
        # Flask app
        self.app = Flask(__name__)
        self.setup_routes()
        
        # Frames for multi-view streaming
        self.frames = {
            'original': None,
            'hsv': None,
            'binary': None,
            'processed': None
        }
        self.frame_lock = threading.Lock()
        
        rospy.loginfo("Green Square Detector initialized")
        rospy.loginfo(f"Loaded green calibration from {self.calibration_file}")

    def load_calibration(self):
        """Load calibration parameters from XML file"""
        try:
            if os.path.exists(self.calibration_file):
                tree = ET.parse(self.calibration_file)
                root = tree.getroot()
                
                # Load HSV range
                hsv_elem = root.find('hsv_range')
                if hsv_elem is not None:
                    for param in ['hue_min', 'hue_max', 'sat_min', 'sat_max', 'val_min', 'val_max']:
                        elem = hsv_elem.find(param)
                        if elem is not None:
                            self.hsv_range[param] = int(elem.text)
                
                # Load alternative HSV range
                hsv2_elem = root.find('hsv_range_2')
                if hsv2_elem is not None:
                    enabled_elem = hsv2_elem.find('enabled')
                    if enabled_elem is not None:
                        self.hsv_range_2['enabled'] = enabled_elem.text.lower() == 'true'
                    for param in ['hue_min', 'hue_max', 'sat_min', 'sat_max', 'val_min', 'val_max']:
                        elem = hsv2_elem.find(param)
                        if elem is not None:
                            self.hsv_range_2[param] = int(elem.text)
                
                # Load visual servoing parameters
                vs_elem = root.find('visual_servoing')
                if vs_elem is not None:
                    for param in ['kp_gain', 'target_size', 'focal_length', 'real_size']:
                        elem = vs_elem.find(param)
                        if elem is not None:
                            self.visual_servoing[param] = float(elem.text)
                
                # Load morphology parameters
                morph_elem = root.find('morphology')
                if morph_elem is not None:
                    for param in ['erosion_iterations', 'dilation_iterations', 'kernel_size', 'min_area']:
                        elem = morph_elem.find(param)
                        if elem is not None:
                            self.morphology[param] = int(elem.text)
                
                # Load detection parameters
                detect_elem = root.find('detection_params')
                if detect_elem is not None:
                    for param in self.detection_params:
                        elem = detect_elem.find(param)
                        if elem is not None:
                            self.detection_params[param] = float(elem.text)
                
                rospy.loginfo("Green calibration loaded successfully from XML")
            else:
                rospy.logwarn(f"Green calibration file {self.calibration_file} not found. Using default values.")
                self.save_calibration()  # Create default file
        except Exception as e:
            rospy.logerr(f"Error loading green calibration: {e}")

    def save_calibration(self):
        """Save current calibration parameters to XML file"""
        try:
            root = ET.Element('green_calibration')
            root.set('version', '1.0')
            root.set('timestamp', str(rospy.Time.now().to_sec()))
            
            # Save HSV range
            hsv_elem = ET.SubElement(root, 'hsv_range')
            for key, value in self.hsv_range.items():
                param_elem = ET.SubElement(hsv_elem, key)
                param_elem.text = str(value)
            
            # Save alternative HSV range
            hsv2_elem = ET.SubElement(root, 'hsv_range_2')
            for key, value in self.hsv_range_2.items():
                param_elem = ET.SubElement(hsv2_elem, key)
                param_elem.text = str(value)
            
            # Save visual servoing parameters
            vs_elem = ET.SubElement(root, 'visual_servoing')
            for key, value in self.visual_servoing.items():
                param_elem = ET.SubElement(vs_elem, key)
                param_elem.text = str(value)
            
            # Save morphology parameters
            morph_elem = ET.SubElement(root, 'morphology')
            for key, value in self.morphology.items():
                param_elem = ET.SubElement(morph_elem, key)
                param_elem.text = str(value)
            
            # Save detection parameters
            detect_elem = ET.SubElement(root, 'detection_params')
            for key, value in self.detection_params.items():
                param_elem = ET.SubElement(detect_elem, key)
                param_elem.text = str(value)
            
            # Write to file with proper formatting
            tree = ET.ElementTree(root)
            ET.indent(tree, space="  ", level=0)
            tree.write(self.calibration_file, encoding='utf-8', xml_declaration=True)
            
            rospy.loginfo(f"Green calibration saved to {self.calibration_file}")
            return True
        except Exception as e:
            rospy.logerr(f"Error saving green calibration: {e}")
            return False

    def setup_routes(self):
        @self.app.route('/')
        def index():
            return render_template('index.html')
        
        @self.app.route('/video_feed/<frame_type>')
        def video_feed(frame_type):
            if frame_type not in ['original', 'hsv', 'binary', 'processed']:
                frame_type = 'original'
            return Response(self.generate_frames(frame_type),
                          mimetype='multipart/x-mixed-replace; boundary=frame')
        
        @self.app.route('/mode_change', methods=['POST'])
        def mode_change():
            data = request.json
            self.current_mode = data.get('mode', 'auto')
            
            # Publish mode change
            msg = String()
            msg.data = json.dumps({
                'type': 'mode_change',
                'mode': self.current_mode,
                'timestamp': data.get('timestamp', 0)
            })
            self.control_pub.publish(msg)
            
            rospy.loginfo(f"Mode changed to: {self.current_mode}")
            return jsonify({'status': 'success', 'mode': self.current_mode})
        
        @self.app.route('/calibration_update', methods=['POST'])
        def calibration_update():
            data = request.json
            self.hsv_range.update(data.get('hsv_range', {}))
            self.hsv_range_2.update(data.get('hsv_range_2', {}))
            self.visual_servoing.update(data.get('visual_servoing', {}))
            self.morphology.update(data.get('morphology', {}))
            self.detection_params.update(data.get('detection_params', {}))
            
            # Publish calibration data
            msg = String()
            msg.data = json.dumps({
                'type': 'calibration_update',
                'hsv_range': self.hsv_range,
                'hsv_range_2': self.hsv_range_2,
                'visual_servoing': self.visual_servoing,
                'morphology': self.morphology,
                'detection_params': self.detection_params,
                'timestamp': data.get('timestamp', 0)
            })
            self.control_pub.publish(msg)
            
            return jsonify({'status': 'success', 'message': 'Green calibration updated'})
        
        @self.app.route('/save_calibration', methods=['POST'])
        def save_calibration_route():
            success = self.save_calibration()
            if success:
                return jsonify({'status': 'success', 'message': 'Green calibration saved to XML'})
            else:
                return jsonify({'status': 'error', 'message': 'Failed to save green calibration'})
        
        @self.app.route('/get_calibration')
        def get_calibration():
            return jsonify({
                'hsv_range': self.hsv_range,
                'hsv_range_2': self.hsv_range_2,
                'visual_servoing': self.visual_servoing,
                'morphology': self.morphology,
                'detection_params': self.detection_params
            })
        
        @self.app.route('/joystick_control', methods=['POST'])
        def joystick_control():
            data = request.json
            self.joystick_data = data.get('joystick', {'x': 0, 'y': 0})
            
            if self.current_mode == 'manual':
                self.publish_manual_control()
            
            return jsonify({'status': 'success', 'joystick': self.joystick_data})
        
        @self.app.route('/keyboard_control', methods=['POST'])
        def keyboard_control():
            data = request.json
            self.keyboard_data = {
                'active_keys': data.get('active_keys', []),
                'command': data.get('command', 'STOP')
            }
            
            if self.current_mode == 'manual':
                self.publish_manual_control()
            
            return jsonify({'status': 'success', 'keyboard': self.keyboard_data})
        
        @self.app.route('/detection_status')
        def detection_status():
            return jsonify(self.detection_data)
        
        @self.app.route('/debug_hsv/<int:x>/<int:y>')
        def debug_hsv(x, y):
            """Debug HSV values at specific pixel coordinates"""
            try:
                ret, frame = self.cap.read()
                if ret:
                    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
                    if 0 <= y < hsv.shape[0] and 0 <= x < hsv.shape[1]:
                        h, s, v = hsv[y, x]
                        return jsonify({
                            'x': x, 'y': y,
                            'hue': int(h), 'sat': int(s), 'val': int(v),
                            'bgr': frame[y, x].tolist()
                        })
                return jsonify({'error': 'Invalid coordinates or no frame'})
            except Exception as e:
                return jsonify({'error': str(e)})
        
        @self.app.route('/mask_debug')
        def mask_debug():
            """Return mask statistics for debugging"""
            try:
                ret, frame = self.cap.read()
                if ret:
                    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
                    
                    # Primary mask
                    lower_green = np.array([
                        self.hsv_range['hue_min'],
                        self.hsv_range['sat_min'],
                        self.hsv_range['val_min']
                    ])
                    upper_green = np.array([
                        self.hsv_range['hue_max'],
                        self.hsv_range['sat_max'],
                        self.hsv_range['val_max']
                    ])
                    mask1 = cv2.inRange(hsv, lower_green, upper_green)
                    
                    # Secondary mask if enabled
                    if self.hsv_range_2.get('enabled', False):
                        lower_green_2 = np.array([
                            self.hsv_range_2['hue_min'],
                            self.hsv_range_2['sat_min'],
                            self.hsv_range_2['val_min']
                        ])
                        upper_green_2 = np.array([
                            self.hsv_range_2['hue_max'],
                            self.hsv_range_2['sat_max'],
                            self.hsv_range_2['val_max']
                        ])
                        mask2 = cv2.inRange(hsv, lower_green_2, upper_green_2)
                        combined_mask = cv2.bitwise_or(mask1, mask2)
                    else:
                        combined_mask = mask1
                    
                    white_pixels = cv2.countNonZero(combined_mask)
                    total_pixels = combined_mask.shape[0] * combined_mask.shape[1]
                    
                    return jsonify({
                        'white_pixels': int(white_pixels),
                        'total_pixels': int(total_pixels),
                        'percentage': float(white_pixels / total_pixels * 100),
                        'primary_pixels': int(cv2.countNonZero(mask1)),
                        'secondary_enabled': self.hsv_range_2.get('enabled', False),
                        'hsv_range': self.hsv_range,
                        'mask_shape': combined_mask.shape
                    })
                return jsonify({'error': 'No frame available'})
            except Exception as e:
                return jsonify({'error': str(e)})

    def detect_green_square(self, frame):
        """Advanced green square detection with dual HSV ranges"""
        original = frame.copy()
        
        # Convert to HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # Primary green range
        lower_green = np.array([
            self.hsv_range['hue_min'],
            self.hsv_range['sat_min'],
            self.hsv_range['val_min']
        ])
        upper_green = np.array([
            self.hsv_range['hue_max'],
            self.hsv_range['sat_max'],
            self.hsv_range['val_max']
        ])
        
        # Create primary mask
        mask1 = cv2.inRange(hsv, lower_green, upper_green)
        
        # Secondary green range if enabled
        if self.hsv_range_2.get('enabled', False):
            lower_green_2 = np.array([
                self.hsv_range_2['hue_min'],
                self.hsv_range_2['sat_min'],
                self.hsv_range_2['val_min']
            ])
            upper_green_2 = np.array([
                self.hsv_range_2['hue_max'],
                self.hsv_range_2['sat_max'],
                self.hsv_range_2['val_max']
            ])
            mask2 = cv2.inRange(hsv, lower_green_2, upper_green_2)
            mask = cv2.bitwise_or(mask1, mask2)  # Combine both masks
            
            # Debug info
            primary_pixels = cv2.countNonZero(mask1)
            secondary_pixels = cv2.countNonZero(mask2)
            combined_pixels = cv2.countNonZero(mask)
            rospy.loginfo_throttle(3, f"Green Detection - Primary: {primary_pixels}, Secondary: {secondary_pixels}, Combined: {combined_pixels}")
        else:
            mask = mask1
        
        # Morphological operations with safety check
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, 
                                         (self.morphology['kernel_size'], 
                                          self.morphology['kernel_size']))
        
        processed = mask.copy()
        
        # Conservative erosion
        if self.morphology['erosion_iterations'] > 0:
            eroded = cv2.erode(processed, kernel, iterations=self.morphology['erosion_iterations'])
            remaining_pixels = cv2.countNonZero(eroded)
            if remaining_pixels > 50:  # Keep erosion if sufficient pixels remain
                processed = eroded
            else:
                rospy.logwarn_throttle(2, f"Erosion too aggressive for green object, skipping. Remaining: {remaining_pixels}")
        
        # Dilation to restore/improve shape
        if self.morphology['dilation_iterations'] > 0:
            processed = cv2.dilate(processed, kernel, iterations=self.morphology['dilation_iterations'])
        
        # Find contours on processed image
        contours, _ = cv2.findContours(processed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        detected = False
        center_x, center_y = 0, 0
        area = 0
        distance = 0
        error_x = 0
        confidence = 0
        
        # Create colored versions for display
        hsv_colored = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
        
        # Enhanced binary mask visualization
        binary_colored = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        binary_colored[mask > 0] = [0, 255, 0]  # Green where mask is white
        
        # Enhanced processed mask visualization
        processed_colored = cv2.cvtColor(processed, cv2.COLOR_GRAY2BGR)
        processed_colored[processed > 0] = [0, 255, 255]  # Cyan where processed is white
        
        if contours:
            # Sort contours by area, get largest
            contours = sorted(contours, key=cv2.contourArea, reverse=True)
            
            for i, contour in enumerate(contours[:3]):  # Check top 3 contours
                area = cv2.contourArea(contour)
                
                # Check minimum area
                if area < self.detection_params['min_contour_area']:
                    continue
                    
                if area > self.detection_params['max_contour_area']:
                    continue
                
                # Get bounding rectangle
                x, y, w, h = cv2.boundingRect(contour)
                
                # Check aspect ratio with flexible bounds
                aspect_ratio = float(w) / h
                if (self.detection_params['min_aspect_ratio'] <= aspect_ratio <= 
                    self.detection_params['max_aspect_ratio']):
                    
                    # Additional shape analysis
                    perimeter = cv2.arcLength(contour, True)
                    if perimeter > 0:
                        circularity = 4 * np.pi * area / (perimeter * perimeter)
                        
                        # Accept shapes that are reasonably square-like
                        if circularity > 0.3:  # More flexible shape requirement
                            detected = True
                            center_x = x + w // 2
                            center_y = y + h // 2
                            
                            # Calculate confidence based on multiple factors
                            area_score = min(1.0, area / 1000.0)  # Normalize area
                            aspect_score = 1.0 - abs(aspect_ratio - 1.0)  # Closer to 1 = higher score
                            circularity_score = circularity
                            confidence = (area_score + aspect_score + circularity_score) / 3.0
                            
                            # Calculate distance using visual servoing
                            pixel_size = max(w, h)
                            if pixel_size > 0:
                                distance = (self.visual_servoing['real_size'] * 
                                          self.visual_servoing['focal_length']) / pixel_size
                            
                            # Calculate error from frame center
                            frame_center_x = frame.shape[1] // 2
                            error_x = center_x - frame_center_x
                            
                            # Draw detection on all frames
                            color = (0, 255, 0)  # Green for detected
                            thickness = 3
                            
                            for img in [original, hsv_colored, binary_colored, processed_colored]:
                                cv2.rectangle(img, (x, y), (x + w, y + h), color, thickness)
                                cv2.circle(img, (center_x, center_y), 8, (0, 0, 255), -1)
                                
                                # Enhanced info display
                                cv2.putText(img, f'Green Square DETECTED', 
                                          (x, y - 50), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
                                cv2.putText(img, f'Conf: {confidence:.2f} AR: {aspect_ratio:.2f}', 
                                          (x, y - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
                                cv2.putText(img, f'Area: {int(area)} Circ: {circularity:.2f}', 
                                          (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
                                cv2.putText(img, f'Distance: {distance:.1f}cm', 
                                          (x, y + h + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
                            
                            break  # Use first valid detection
                    
                # Draw rejected contours for debugging
                if not detected and i < 2:  # Only show first 2 rejected
                    x, y, w, h = cv2.boundingRect(contour)
                    aspect_ratio = float(w) / h
                    for img in [original, hsv_colored, binary_colored, processed_colored]:
                        cv2.rectangle(img, (x, y), (x + w, y + h), (0, 0, 255), 1)  # Red for rejected
                        cv2.putText(img, f'REJECTED AR: {aspect_ratio:.2f}', 
                                  (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1)
        
        # Update detection data
        self.detection_data = {
            'detected': detected,
            'center_x': int(center_x),
            'center_y': int(center_y),
            'area': int(area),
            'distance': float(distance),
            'error_x': float(error_x),
            'confidence': float(confidence)
        }
        
        # Draw center crosshair on all frames
        h, w = frame.shape[:2]
        for img in [original, hsv_colored, binary_colored, processed_colored]:
            cv2.line(img, (w//2 - 20, h//2), (w//2 + 20, h//2), (255, 255, 0), 2)
            cv2.line(img, (w//2, h//2 - 20), (w//2, h//2 + 20), (255, 255, 0), 2)
            
            # Add frame-specific labels
            labels = {
                original: "RGB Original",
                hsv_colored: "HSV Color Space", 
                binary_colored: "Green Binary Mask",
                processed_colored: "Morphology Result"
            }
            
            label = labels.get(id(img), "Unknown")
            cv2.putText(img, label, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        # Store frames for streaming
        with self.frame_lock:
            self.frames['original'] = original
            self.frames['hsv'] = hsv_colored
            self.frames['binary'] = binary_colored
            self.frames['processed'] = processed_colored
        
        return original, detected

    def publish_visual_servoing_control(self):
        """Publish control commands based on visual servoing"""
        if not self.detection_data['detected']:
            # Stop if no detection
            twist = Twist()
            self.cmd_vel_pub.publish(twist)
            return
        
        # Calculate control signals
        error_x = self.detection_data['error_x']
        distance = self.detection_data['distance']
        confidence = self.detection_data['confidence']
        target_distance = self.visual_servoing['real_size'] * 4  # 4x object size as target
        
        # Confidence-based control scaling
        kp = self.visual_servoing['kp_gain'] * confidence  # Scale by confidence
        
        # Angular velocity (yaw) to center the object
        angular_z = -kp * error_x / 100.0  # Normalize error
        
        # Linear velocity (forward/backward) based on distance error
        distance_error = distance - target_distance
        linear_x = -kp * distance_error / 50.0  # Normalize distance error
        
        # Limit velocities
        angular_z = max(-0.8, min(0.8, angular_z))  # Slightly lower max speed
        linear_x = max(-0.4, min(0.4, linear_x))    # Slightly lower max speed
        
        # Publish twist command
        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        self.cmd_vel_pub.publish(twist)
        
        # Publish control data
        msg = String()
        msg.data = json.dumps({
            'type': 'manual_control',
            'joystick': self.joystick_data,
            'keyboard': self.keyboard_data,
            'control': {'linear_x': linear_x, 'angular_z': angular_z}
        })
        self.control_pub.publish(msg)

    def generate_frames(self, frame_type='original'):
        """Generate video frames for streaming"""
        while True:
            ret, frame = self.cap.read()
            if not ret:
                break
            
            # Process frame based on current mode
            if self.current_mode == 'auto':
                processed_frame, detected = self.detect_green_square(frame)
                if detected and self.detection_data['confidence'] > 0.3:  # Confidence threshold
                    self.publish_visual_servoing_control()
            else:
                # For manual mode, still process for display but don't control
                self.detect_green_square(frame)
            
            # Get the requested frame type
            with self.frame_lock:
                if frame_type in self.frames and self.frames[frame_type] is not None:
                    display_frame = self.frames[frame_type].copy()
                else:
                    display_frame = frame.copy()
            
            # Add mode indicator
            mode_color = (0, 255, 0) if self.current_mode == 'auto' else (255, 255, 0)
            cv2.putText(display_frame, f'Mode: {self.current_mode.upper()}', 
                       (10, display_frame.shape[0] - 60), cv2.FONT_HERSHEY_SIMPLEX, 1, mode_color, 2)
            
            # Add detection status
            if self.detection_data['detected']:
                status_text = f"GREEN DETECTED - Conf: {self.detection_data['confidence']:.2f}"
                status_color = (0, 255, 0)
            else:
                status_text = "SEARCHING FOR GREEN SQUARE"
                status_color = (0, 0, 255)
            
            cv2.putText(display_frame, status_text, 
                       (10, display_frame.shape[0] - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, status_color, 2)
            
            # Add frame type indicator
            frame_labels = {
                'original': 'RGB Original',
                'hsv': 'HSV Color Space',
                'binary': 'Green Binary Mask',
                'processed': 'Morphology Result'
            }
            cv2.putText(display_frame, frame_labels.get(frame_type, frame_type), 
                       (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            
            # Encode frame
            ret, buffer = cv2.imencode('.jpg', display_frame)
            if ret:
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')

    def run_flask(self):
        """Run Flask server"""
        self.app.run(host='0.0.0.0', port=5000, debug=False, threaded=True)

    def run(self):
        """Main run function"""
        try:
            # Start Flask in a separate thread
            flask_thread = threading.Thread(target=self.run_flask)
            flask_thread.daemon = True
            flask_thread.start()
            
            rospy.loginfo("Flask server started on http://0.0.0.0:5000")
            rospy.loginfo("Green Square Detector node running...")
            
            # Keep ROS node alive
            rospy.spin()
            
        except KeyboardInterrupt:
            rospy.loginfo("Shutting down Green Square Detector")
        finally:
            if self.cap:
                self.cap.release()
            cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        detector = GreenSquareDetector()
        detector.run()
    except rospy.ROSInterruptException:
        pass
        
        # Publish detection data
        msg = String()
        msg.data = json.dumps({
            'type': 'green_visual_servoing',
            'detection': self.detection_data,
            'control': {'linear_x': linear_x, 'angular_z': angular_z},
            'confidence': confidence
        })
        self.detection_pub.publish(msg)

    def publish_manual_control(self):
        """Publish manual control commands"""
        twist = Twist()
        
        # Combine joystick and keyboard inputs
        linear_x = self.joystick_data['y'] / 100.0  # Normalize joystick Y to -1,1
        angular_z = -self.joystick_data['x'] / 100.0  # Normalize joystick X to -1,1
        
        # Override with keyboard if active
        if self.keyboard_data['active_keys']:
            linear_x = 0
            angular_z = 0
            
            if 'w' in self.keyboard_data['active_keys']:
                linear_x += 0.4
            if 's' in self.keyboard_data['active_keys']:
                linear_x -= 0.4
            if 'a' in self.keyboard_data['active_keys']:
                angular_z += 0.4
            if 'd' in self.keyboard_data['active_keys']:
                angular_z -= 0.4
        
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        self.cmd_vel_pub.publish(twist)