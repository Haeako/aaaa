import numpy as np
cimport numpy as np
from libc.stdlib cimport malloc, free
from libcpp cimport bool

# Bật chế độ biên dịch nhanh
cdef extern from "math.h":
    double sqrt(double x)

# Tăng tốc chuyển đổi ảnh bằng NumPy
cpdef np.ndarray filter_image(np.ndarray[np.uint8_t, ndim=3] frame):
    cdef int h = frame.shape[0]
    cdef int w = frame.shape[1]

    # Chuyển sang HSV
    frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Áp dụng threshold
    lower = np.array([30, 120, 50], dtype=np.uint8)
    upper = np.array([87, 255, 255], dtype=np.uint8)
    frame_thresh = cv2.inRange(frame_hsv, lower, upper)

    return frame_thresh

# Tăng tốc tìm contours (chạy mà không giữ GIL)
cdef bool find_best_contour(np.ndarray[np.uint8_t, ndim=2] binary_img, int* center_x, int* center_y) nogil:
    cdef list contours
    cdef int i
    cdef int best_index = -1
    cdef double max_area = 0.0
    cdef double area
    cdef double cx, cy
    cdef np.ndarray[np.uint8_t, ndim=3] contour_img

    # Tìm contours
    contours, _ = cv2.findContours(binary_img, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
    
    for i in range(len(contours)):
        area = cv2.contourArea(contours[i])
        
        # Chỉ lấy contour có diện tích phù hợp
        if 2500 < area < 57600 and area > max_area:
            max_area = area
            best_index = i
    
    # Nếu tìm thấy contour phù hợp
    if best_index >= 0:
        M = cv2.moments(contours[best_index])
        cx = M["m10"] / (M["m00"] + 1e-5)
        cy = M["m01"] / (M["m00"] + 1e-5)
        center_x[0] = <int> cx
        center_y[0] = <int> cy
        return True

    return False
