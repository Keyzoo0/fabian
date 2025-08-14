import cv2
import numpy as np
import yaml
import os

# Set X11 forwarding optimization
os.environ['QT_X11_NO_MITSHM'] = '1'
os.environ['DISPLAY'] = ':0'  # Fallback display

# Check if running via SSH and setup display
def setup_display():
    """Setup display for SSH X11 forwarding"""
    display = os.environ.get('DISPLAY')
    
    if not display:
        print("No DISPLAY environment variable found.")
        print("Solutions:")
        print("1. Reconnect SSH with: ssh -X username@hostname")
        print("2. Or set display manually: export DISPLAY=:0")
        return False
    
    print(f"DISPLAY: {display}")
    return True

# Test X11 connection
def test_x11():
    """Test if X11 forwarding is working"""
    try:
        # Try to create a simple window to test X11
        import subprocess
        result = subprocess.run(['xset', 'q'], capture_output=True, text=True, timeout=5)
        if result.returncode == 0:
            print("X11 forwarding is working")
            return True
        else:
            print("X11 forwarding test failed")
            return False
    except Exception as e:
        print(f"X11 test error: {e}")
        return False

# Fungsi callback untuk trackbar (tidak perlu implementasi)
def nothing(x):
    pass

# File untuk menyimpan konfigurasi
CONFIG_FILE = 'color_detection_config.yaml'

# Fungsi untuk menyimpan konfigurasi ke YAML
def save_config():
    config = {
        'hsv': {
            'h_min': cv2.getTrackbarPos('H Min', 'Trackbar'),
            's_min': cv2.getTrackbarPos('S Min', 'Trackbar'),
            'v_min': cv2.getTrackbarPos('V Min', 'Trackbar'),
            'h_max': cv2.getTrackbarPos('H Max', 'Trackbar'),
            's_max': cv2.getTrackbarPos('S Max', 'Trackbar'),
            'v_max': cv2.getTrackbarPos('V Max', 'Trackbar')
        },
        'morphology': {
            'erosion': cv2.getTrackbarPos('Erosion', 'Trackbar'),
            'dilation': cv2.getTrackbarPos('Dilation', 'Trackbar'),
            'opening': cv2.getTrackbarPos('Opening', 'Trackbar'),
            'closing': cv2.getTrackbarPos('Closing', 'Trackbar')
        },
        'calibration': {
            'calib_distance_cm': cv2.getTrackbarPos('Calib Distance (cm)', 'Trackbar'),
            'calib_pixel_area': cv2.getTrackbarPos('Calib Pixel Area', 'Trackbar'),
            'min_area_threshold': cv2.getTrackbarPos('Min Area Threshold', 'Trackbar')
        },
        'camera_controls': {
            'auto_exposure': cv2.getTrackbarPos('Auto Exposure', 'Trackbar'),
            'exposure': cv2.getTrackbarPos('Exposure', 'Trackbar'),
            'auto_wb': cv2.getTrackbarPos('Auto White Balance', 'Trackbar'),
            'wb_temperature': cv2.getTrackbarPos('WB Temperature', 'Trackbar'),
            'brightness': cv2.getTrackbarPos('Brightness', 'Trackbar'),
            'contrast': cv2.getTrackbarPos('Contrast', 'Trackbar')
        }
    }
    
    with open(CONFIG_FILE, 'w') as file:
        yaml.dump(config, file, default_flow_style=False)
    print(f"Configuration saved to {CONFIG_FILE}")

# Fungsi untuk memuat konfigurasi dari YAML
def load_config():
    if os.path.exists(CONFIG_FILE):
        try:
            with open(CONFIG_FILE, 'r') as file:
                config = yaml.safe_load(file)
            print(f"Configuration loaded from {CONFIG_FILE}")
            return config
        except Exception as e:
            print(f"Error loading config: {e}")
            return None
    else:
        print(f"Config file {CONFIG_FILE} not found, using default values")
        return None

# Fungsi untuk set trackbar dari config
def set_trackbars_from_config(config):
    if config:
        # Set HSV trackbars dengan default values
        cv2.setTrackbarPos('H Min', 'Trackbar', config.get('hsv', {}).get('h_min', 0))
        cv2.setTrackbarPos('S Min', 'Trackbar', config.get('hsv', {}).get('s_min', 0))
        cv2.setTrackbarPos('V Min', 'Trackbar', config.get('hsv', {}).get('v_min', 0))
        cv2.setTrackbarPos('H Max', 'Trackbar', config.get('hsv', {}).get('h_max', 179))
        cv2.setTrackbarPos('S Max', 'Trackbar', config.get('hsv', {}).get('s_max', 255))
        cv2.setTrackbarPos('V Max', 'Trackbar', config.get('hsv', {}).get('v_max', 255))
        
        # Set morphology trackbars dengan default values
        cv2.setTrackbarPos('Erosion', 'Trackbar', config.get('morphology', {}).get('erosion', 0))
        cv2.setTrackbarPos('Dilation', 'Trackbar', config.get('morphology', {}).get('dilation', 0))
        cv2.setTrackbarPos('Opening', 'Trackbar', config.get('morphology', {}).get('opening', 0))
        cv2.setTrackbarPos('Closing', 'Trackbar', config.get('morphology', {}).get('closing', 0))
        
        # Set calibration trackbars dengan default values
        cv2.setTrackbarPos('Calib Distance (cm)', 'Trackbar', config.get('calibration', {}).get('calib_distance_cm', 50))
        cv2.setTrackbarPos('Calib Pixel Area', 'Trackbar', config.get('calibration', {}).get('calib_pixel_area', 5000))
        cv2.setTrackbarPos('Min Area Threshold', 'Trackbar', config.get('calibration', {}).get('min_area_threshold', 500))
        
        # Set camera control trackbars dengan default values
        cv2.setTrackbarPos('Auto Exposure', 'Trackbar', config.get('camera_controls', {}).get('auto_exposure', 0))
        cv2.setTrackbarPos('Exposure', 'Trackbar', config.get('camera_controls', {}).get('exposure', 50))
        cv2.setTrackbarPos('Auto White Balance', 'Trackbar', config.get('camera_controls', {}).get('auto_wb', 0))
        cv2.setTrackbarPos('WB Temperature', 'Trackbar', config.get('camera_controls', {}).get('wb_temperature', 40))
        cv2.setTrackbarPos('Brightness', 'Trackbar', config.get('camera_controls', {}).get('brightness', 50))
        cv2.setTrackbarPos('Contrast', 'Trackbar', config.get('camera_controls', {}).get('contrast', 50))

# Check display dan X11 sebelum memulai
if not setup_display():
    print("Exiting due to display setup failure")
    exit(1)

if not test_x11():
    print("X11 forwarding not working properly")
    print("Try these solutions:")
    print("1. ssh -Y username@hostname  (trusted X11 forwarding)")
    print("2. Install xauth: sudo apt install xauth")
    print("3. Check SSH config: X11Forwarding yes")
    exit(1)

# Inisialisasi kamera
cap = cv2.VideoCapture(0)

# Optimasi kamera untuk low latency
cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # Reduce buffer size
cap.set(cv2.CAP_PROP_FPS, 30)        # Set FPS

# Dapatkan resolusi kamera
ret, test_frame = cap.read()
if ret:
    frame_height, frame_width = test_frame.shape[:2]
    max_pixel_area = (frame_width * frame_height) // 2  # Setengah dari total area frame
    print(f"Camera resolution: {frame_width}x{frame_height}")
    print(f"Max pixel area: {max_pixel_area}")
else:
    frame_width, frame_height = 640, 480  # Default fallback
    max_pixel_area = (frame_width * frame_height) // 2
    print("Using default resolution: 640x480")

# Buat window untuk trackbar dengan optimasi
cv2.namedWindow('Trackbar', cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO)
cv2.resizeWindow('Trackbar', 400, 600)

# Buat trackbar untuk HSV
cv2.createTrackbar('H Min', 'Trackbar', 0, 179, nothing)
cv2.createTrackbar('S Min', 'Trackbar', 0, 255, nothing)
cv2.createTrackbar('V Min', 'Trackbar', 0, 255, nothing)
cv2.createTrackbar('H Max', 'Trackbar', 179, 179, nothing)
cv2.createTrackbar('S Max', 'Trackbar', 255, 255, nothing)
cv2.createTrackbar('V Max', 'Trackbar', 255, 255, nothing)

# Buat trackbar untuk morphology operations
cv2.createTrackbar('Erosion', 'Trackbar', 0, 10, nothing)
cv2.createTrackbar('Dilation', 'Trackbar', 0, 10, nothing)
cv2.createTrackbar('Opening', 'Trackbar', 0, 10, nothing)
cv2.createTrackbar('Closing', 'Trackbar', 0, 10, nothing)

# Buat trackbar untuk kalibrasi jarak
cv2.createTrackbar('Calib Distance (cm)', 'Trackbar', 50, 200, nothing)  # Default: 50 cm
cv2.createTrackbar('Calib Pixel Area', 'Trackbar', 5000, max_pixel_area, nothing)  # Max: setengah area frame
cv2.createTrackbar('Min Area Threshold', 'Trackbar', 500, max_pixel_area, nothing)  # Default: 500 pixels

# Buat trackbar untuk camera controls
cv2.createTrackbar('Auto Exposure', 'Trackbar', 0, 1, nothing)        # 0=Manual, 1=Auto
cv2.createTrackbar('Exposure', 'Trackbar', 50, 100, nothing)           # 0-100 (mapped to actual range)
cv2.createTrackbar('Auto White Balance', 'Trackbar', 0, 1, nothing)    # 0=Manual, 1=Auto
cv2.createTrackbar('WB Temperature', 'Trackbar', 40, 80, nothing)      # 2000-8000K (mapped)
cv2.createTrackbar('Brightness', 'Trackbar', 50, 100, nothing)         # 0-100
cv2.createTrackbar('Contrast', 'Trackbar', 50, 100, nothing)           # 0-100

# Load konfigurasi yang tersimpan
config = load_config()
set_trackbars_from_config(config)

# Buat window untuk display dengan optimasi low latency
cv2.namedWindow('Color Detection', cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO)

# Resize frame untuk mengurangi latency (optional)
SCALE_FACTOR = 0.75  # Reduce to 75% for faster transmission
display_width = int(frame_width * 2 * SCALE_FACTOR)  # 2x karena hstack
display_height = int(frame_height * 2 * SCALE_FACTOR)  # 2x karena vstack

print("SSH X11 Forwarding Mode")
print("Connect with: ssh -X username@hostname")
print("Press 'S' to Save Config | Press 'Q' to Quit")

while True:
    ret, frame = cap.read()
    if not ret:
        break
    
    # Konversi BGR ke HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # Ambil nilai dari trackbar
    h_min = cv2.getTrackbarPos('H Min', 'Trackbar')
    s_min = cv2.getTrackbarPos('S Min', 'Trackbar')
    v_min = cv2.getTrackbarPos('V Min', 'Trackbar')
    h_max = cv2.getTrackbarPos('H Max', 'Trackbar')
    s_max = cv2.getTrackbarPos('S Max', 'Trackbar')
    v_max = cv2.getTrackbarPos('V Max', 'Trackbar')
    
    # Ambil nilai morphology dari trackbar
    erosion_size = cv2.getTrackbarPos('Erosion', 'Trackbar')
    dilation_size = cv2.getTrackbarPos('Dilation', 'Trackbar')
    opening_size = cv2.getTrackbarPos('Opening', 'Trackbar')
    closing_size = cv2.getTrackbarPos('Closing', 'Trackbar')
    
    # Ambil nilai kalibrasi jarak
    calib_distance_cm = cv2.getTrackbarPos('Calib Distance (cm)', 'Trackbar')
    calib_pixel_area = cv2.getTrackbarPos('Calib Pixel Area', 'Trackbar')
    min_area_threshold = cv2.getTrackbarPos('Min Area Threshold', 'Trackbar')
    
    # Ambil nilai camera controls
    auto_exposure = cv2.getTrackbarPos('Auto Exposure', 'Trackbar')
    exposure_val = cv2.getTrackbarPos('Exposure', 'Trackbar')
    auto_wb = cv2.getTrackbarPos('Auto White Balance', 'Trackbar')
    wb_temp = cv2.getTrackbarPos('WB Temperature', 'Trackbar')
    brightness = cv2.getTrackbarPos('Brightness', 'Trackbar')
    contrast = cv2.getTrackbarPos('Contrast', 'Trackbar')
    
    # Apply camera settings
    try:
        # Auto Exposure (0=Manual, 1=Auto)
        if auto_exposure == 0:
            cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)  # Manual mode
            # Map 0-100 to exposure range (biasanya -13 to -1)
            exposure_mapped = -13 + (exposure_val / 100.0) * 12
            cap.set(cv2.CAP_PROP_EXPOSURE, exposure_mapped)
        else:
            cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.75)  # Auto mode
        
        # White Balance
        cap.set(cv2.CAP_PROP_AUTO_WB, auto_wb)
        if auto_wb == 0:  # Manual WB
            # Map 0-80 to 2000K-8000K
            wb_temp_mapped = 2000 + (wb_temp / 80.0) * 6000
            cap.set(cv2.CAP_PROP_WB_TEMPERATURE, wb_temp_mapped)
        
        # Brightness and Contrast (0-100 range)
        cap.set(cv2.CAP_PROP_BRIGHTNESS, brightness)
        cap.set(cv2.CAP_PROP_CONTRAST, contrast)
        
    except Exception as e:
        pass  # Some cameras might not support all controls
    
    # Buat range HSV
    lower = np.array([h_min, s_min, v_min])
    upper = np.array([h_max, s_max, v_max])
    
    # Buat mask
    mask = cv2.inRange(hsv, lower, upper)
    
    # Terapkan morphology operations pada mask
    if erosion_size > 0:
        kernel_erosion = np.ones((erosion_size, erosion_size), np.uint8)
        mask = cv2.erode(mask, kernel_erosion, iterations=1)
    
    if dilation_size > 0:
        kernel_dilation = np.ones((dilation_size, dilation_size), np.uint8)
        mask = cv2.dilate(mask, kernel_dilation, iterations=1)
    
    if opening_size > 0:
        kernel_opening = np.ones((opening_size, opening_size), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel_opening)
    
    if closing_size > 0:
        kernel_closing = np.ones((closing_size, closing_size), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel_closing)
    
    # Terapkan mask ke frame asli
    result = cv2.bitwise_and(frame, frame, mask=mask)
    
    # Cari kontur dan gambar bounding box
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    # Buat copy frame untuk bounding box
    frame_with_box = frame.copy()
    
    # Tambahkan garis tengah X dan Y (crosshair di center frame)
    frame_center_x, frame_center_y = frame.shape[1] // 2, frame.shape[0] // 2
    
    # Garis horizontal (kiri ke kanan)
    cv2.line(frame_with_box, (0, frame_center_y), (frame.shape[1], frame_center_y), (255, 255, 0), 1)
    # Garis vertikal (atas ke bawah)  
    cv2.line(frame_with_box, (frame_center_x, 0), (frame_center_x, frame.shape[0]), (255, 255, 0), 1)
    
    # Tambahkan titik center
    cv2.circle(frame_with_box, (frame_center_x, frame_center_y), 3, (255, 255, 0), -1)
    
    if contours:
        # Cari kontur dengan area terbesar
        largest_contour = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest_contour)
        
        # Hanya gambar bounding box jika area cukup besar (filter noise)
        if area > min_area_threshold:  # threshold area minimum dari trackbar
            x, y, w, h = cv2.boundingRect(largest_contour)
            
            # Hitung jarak berdasarkan kalibrasi (menggunakan area)
            current_pixel_area = area
            
            if calib_pixel_area > 0 and calib_distance_cm > 0:
                estimated_distance = calib_distance_cm * (calib_pixel_area / current_pixel_area) ** 0.5
            else:
                estimated_distance = 0  # Belum dikalibrasi
            
            # Gambar bounding box
            cv2.rectangle(frame_with_box, (x, y), (x + w, y + h), (0, 255, 0), 2)
            
            # Tambahkan text info area dan jarak
            cv2.putText(frame_with_box, f'Area: {int(area)} px', (x, y - 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            cv2.putText(frame_with_box, f'Distance: {estimated_distance:.1f} cm', (x, y - 10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
            
            # Tambahkan crosshair di tengah objek
            center_x, center_y = x + w//2, y + h//2
            cv2.circle(frame_with_box, (center_x, center_y), 5, (255, 0, 0), -1)
            cv2.line(frame_with_box, (center_x-10, center_y), (center_x+10, center_y), (255, 0, 0), 2)
            cv2.line(frame_with_box, (center_x, center_y-10), (center_x, center_y+10), (255, 0, 0), 2)
            
            # Hitung error jarak dari center frame
            error_x = center_x - frame_center_x  # Positif = kanan, Negatif = kiri
            error_y = center_y - frame_center_y  # Positif = bawah, Negatif = atas
            
            # Gambar garis dari center frame ke center objek
            cv2.line(frame_with_box, (frame_center_x, frame_center_y), (center_x, center_y), (0, 255, 255), 2)
            
            # Tambahkan text info error X dan Y
            cv2.putText(frame_with_box, f'Error X: {error_x:+d} px', (x, y - 50), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
            cv2.putText(frame_with_box, f'Error Y: {error_y:+d} px', (x, y - 70), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
    
    # Konversi mask ke 3 channel untuk penggabungan
    mask_3ch = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
    
    # Gabungkan 4 frame menjadi 1 frame besar
    top_row = np.hstack((frame_with_box, hsv))
    bottom_row = np.hstack((mask_3ch, result))
    combined = np.vstack((top_row, bottom_row))
    
    # Resize untuk mengurangi latency transmisi via SSH
    combined_resized = cv2.resize(combined, (display_width, display_height), interpolation=cv2.INTER_LINEAR)
    
    # Tambahkan label text pada setiap frame (adjust untuk resize)
    font_scale = SCALE_FACTOR * 0.8
    thickness = max(1, int(SCALE_FACTOR * 2))
    
    cv2.putText(combined_resized, 'Original + BBox', (10, int(25*SCALE_FACTOR)), 
                cv2.FONT_HERSHEY_SIMPLEX, font_scale, (255, 255, 255), thickness)
    cv2.putText(combined_resized, 'HSV', (int((frame.shape[1] + 10)*SCALE_FACTOR), int(25*SCALE_FACTOR)), 
                cv2.FONT_HERSHEY_SIMPLEX, font_scale, (255, 255, 255), thickness)
    cv2.putText(combined_resized, 'Mask', (10, int((frame.shape[0] + 25)*SCALE_FACTOR)), 
                cv2.FONT_HERSHEY_SIMPLEX, font_scale, (255, 255, 255), thickness)
    cv2.putText(combined_resized, 'Result', (int((frame.shape[1] + 10)*SCALE_FACTOR), int((frame.shape[0] + 25)*SCALE_FACTOR)), 
                cv2.FONT_HERSHEY_SIMPLEX, font_scale, (255, 255, 255), thickness)
    
    # Tambahkan info kalibrasi di frame
    info_text = f'Calibration: {calib_distance_cm}cm distance = {calib_pixel_area}px area'
    cv2.putText(combined_resized, info_text, (10, int((combined.shape[0] - 35)*SCALE_FACTOR)), 
               cv2.FONT_HERSHEY_SIMPLEX, font_scale*0.7, (0, 255, 255), thickness)
    
    # Tambahkan instruksi kontrol
    control_text = "Press 'S' to Save Config | Press 'Q' to Quit"
    cv2.putText(combined_resized, control_text, (10, int((combined.shape[0] - 15)*SCALE_FACTOR)), 
               cv2.FONT_HERSHEY_SIMPLEX, font_scale*0.7, (255, 255, 0), thickness)
    
    # Tampilkan frame gabungan
    cv2.imshow('Color Detection', combined_resized)
    
    # Keluar jika tekan 'q' atau simpan config jika tekan 's'
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break
    elif key == ord('s'):
        save_config()

# Bersihkan
cap.release()
cv2.destroyAllWindows()