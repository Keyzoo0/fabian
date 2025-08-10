import cv2
import numpy as np

# Fungsi callback untuk trackbar (tidak perlu implementasi)
def nothing(x):
    pass

# Inisialisasi kamera
cap = cv2.VideoCapture(2)

# Buat window untuk trackbar
cv2.namedWindow('Trackbar')
cv2.resizeWindow('Trackbar', 400, 300)

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
    
    if contours:
        # Cari kontur dengan area terbesar
        largest_contour = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest_contour)
        
        # Hanya gambar bounding box jika area cukup besar (filter noise)
        if area > 500:  # threshold area minimum
            x, y, w, h = cv2.boundingRect(largest_contour)
            
            # Gambar bounding box
            cv2.rectangle(frame_with_box, (x, y), (x + w, y + h), (0, 255, 0), 2)
            
            # Tambahkan text info area
            cv2.putText(frame_with_box, f'Area: {int(area)}', (x, y - 10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
    
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
    
    # Tampilkan frame gabungan
    cv2.imshow('Color Detection', combined)
    
    # Keluar jika tekan 'q'
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Bersihkan
cap.release()
cv2.destroyAllWindows()