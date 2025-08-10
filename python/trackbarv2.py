import cv2
import numpy as np
import yaml
import os

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
            'calib_pixel_area': cv2.getTrackbarPos('Calib Pixel Area', 'Trackbar')
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

# Inisialisasi kamera
cap = cv2.VideoCapture(2)

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

# Buat window untuk trackbar
cv2.namedWindow('Trackbar')
cv2.resizeWindow('Trackbar', 400, 400)

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

# Load konfigurasi yang tersimpan
config = load_config()
set_trackbars_from_config(config)

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
        if area > 500:  # threshold area minimum
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
    
    # Tambahkan label text pada setiap frame
    cv2.putText(combined, 'Original + BBox', (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    cv2.putText(combined, 'HSV', (frame.shape[1] + 10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    cv2.putText(combined, 'Mask', (10, frame.shape[0] + 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    cv2.putText(combined, 'Result', (frame.shape[1] + 10, frame.shape[0] + 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    
    # Tambahkan info kalibrasi di frame
    info_text = f'Calibration: {calib_distance_cm}cm distance = {calib_pixel_area}px area'
    cv2.putText(combined, info_text, (10, combined.shape[0] - 35), 
               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
    
    # Tambahkan instruksi kontrol
    control_text = "Press 'S' to Save Config | Press 'Q' to Quit"
    cv2.putText(combined, control_text, (10, combined.shape[0] - 15), 
               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
    
    # Tampilkan frame gabungan
    cv2.imshow('Color Detection', combined)
    
    # Keluar jika tekan 'q' atau simpan config jika tekan 's'
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break
    elif key == ord('s'):
        save_config()

# Bersihkan
cap.release()
cv2.destroyAllWindows()