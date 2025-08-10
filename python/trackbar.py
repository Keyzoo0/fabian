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
    
    # Buat range HSV
    lower = np.array([h_min, s_min, v_min])
    upper = np.array([h_max, s_max, v_max])
    
    # Buat mask
    mask = cv2.inRange(hsv, lower, upper)
    
    # Terapkan mask ke frame asli
    result = cv2.bitwise_and(frame, frame, mask=mask)
    
    # Konversi mask ke 3 channel untuk penggabungan
    mask_3ch = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
    
    # Gabungkan 4 frame menjadi 1 frame besar
    top_row = np.hstack((frame, hsv))
    bottom_row = np.hstack((mask_3ch, result))
    combined = np.vstack((top_row, bottom_row))
    
    # Tambahkan label text pada setiap frame
    cv2.putText(combined, 'Original', (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
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