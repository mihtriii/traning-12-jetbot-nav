import cv2
import numpy as np

# ==============================================================================
# PHẦN 1: COPY-PASTE CÁC THAM SỐ TỪ CODE CHÍNH
# Hãy đảm bảo các giá trị này giống hệt như trong file controller.
# ==============================================================================

# Kích thước ảnh mà robot xử lý
WIDTH, HEIGHT = 300, 300

# Tham số ROI
ROI_Y = int(HEIGHT * 0.85)
ROI_H = int(HEIGHT * 0.15)
ROI_CENTER_WIDTH_PERCENT = 0.5

# Tham số ROI Dự báo
LOOKAHEAD_ROI_Y = int(HEIGHT * 0.60)
LOOKAHEAD_ROI_H = int(HEIGHT * 0.15)

# Dải màu HSV cho vạch đen
LINE_COLOR_LOWER = np.array([0, 0, 0])
LINE_COLOR_UPPER = np.array([180, 255, 60])

# Ngưỡng diện tích contour
SCAN_PIXEL_THRESHOLD = 100

# ==============================================================================
# PHẦN 2: COPY-PASTE HÀM _get_line_center TỪ CODE CHÍNH
# Hàm này được sửa đổi một chút để hiển thị các bước trung gian.
# ==============================================================================

def visualize_get_line_center(image, roi_y, roi_h, window_name_prefix=""):
    """
    Phiên bản gỡ lỗi của _get_line_center.
    Thực hiện xử lý và hiển thị từng bước.
    """
    print(f"\n--- Bắt đầu xử lý cho {window_name_prefix} ---")

    if image is None: 
        print("Lỗi: Không có ảnh đầu vào.")
        return None
    
    # --- BƯỚC 1: Cắt vùng quan tâm (ROI) ---
    roi_original = image[roi_y : roi_y + roi_h, :]
    cv2.imshow(f"{window_name_prefix} Step 1 - ROI Original", roi_original)
    print("Bước 1: Đã cắt ROI.")
    
    # --- BƯỚC 2: Chuyển đổi sang HSV ---
    hsv = cv2.cvtColor(roi_original, cv2.COLOR_BGR2HSV)
    print("Bước 2: Đã chuyển sang HSV.")
    
    # --- BƯỚC 3: Tạo Mặt nạ Màu sắc ---
    color_mask = cv2.inRange(hsv, LINE_COLOR_LOWER, LINE_COLOR_UPPER)
    cv2.imshow(f"{window_name_prefix} Step 3 - Color Mask (Tim mau den)", color_mask)
    print("Bước 3: Đã tạo mặt nạ màu đen.")
    
    # --- BƯỚC 4: Tạo Mặt nạ Tập trung ---
    focus_mask = np.zeros_like(color_mask)
    roi_height, roi_width = focus_mask.shape
    center_width = int(roi_width * ROI_CENTER_WIDTH_PERCENT)
    start_x = (roi_width - center_width) // 2
    end_x = start_x + center_width
    cv2.rectangle(focus_mask, (start_x, 0), (end_x, roi_height), 255, -1)
    cv2.imshow(f"{window_name_prefix} Step 4 - Focus Mask (Vung tap trung)", focus_mask)
    print("Bước 4: Đã tạo mặt nạ tập trung.")
    
    # --- BƯỚC 5: Kết hợp hai mặt nạ ---
    final_mask = cv2.bitwise_and(color_mask, focus_mask)
    cv2.imshow(f"{window_name_prefix} Step 5 - Final Mask (Ket qua loc)", final_mask)
    print("Bước 5: Đã kết hợp hai mặt nạ.")
    
    # --- BƯỚC 6: Tìm Contours ---
    # Sử dụng `_, contours, _` để tương thích OpenCV 3
    contours, _ = cv2.findContours(final_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    print(f"Bước 6: Tìm thấy {len(contours)} contour(s).")
    
    if not contours:
        print("KẾT QUẢ: Không tìm thấy line.")
        return None
        
    c = max(contours, key=cv2.contourArea)
    contour_area = cv2.contourArea(c)
    print(f"  -> Contour lớn nhất có diện tích: {contour_area} (Ngưỡng: {SCAN_PIXEL_THRESHOLD})")

    # Vẽ contour tìm được lên ảnh ROI gốc để trực quan hóa
    roi_with_contour = roi_original.copy()
    cv2.drawContours(roi_with_contour, [c], -1, (0, 255, 0), 2) # Vẽ màu xanh lá

    if contour_area < SCAN_PIXEL_THRESHOLD:
        print("KẾT QUẢ: Contour quá nhỏ, bị loại bỏ.")
        cv2.imshow(f"{window_name_prefix} Step 7 - Result", roi_with_contour)
        return None

    # --- BƯỚC 7: Tính toán và Hiển thị Trọng tâm ---
    M = cv2.moments(c)
    line_center_x = None
    if M["m00"] > 0:
        line_center_x = int(M["m10"] / M["m00"])
        print(f"Bước 7: Đã tính toán trọng tâm tại x = {line_center_x}")
        # Vẽ một vòng tròn đỏ tại trọng tâm
        cv2.circle(roi_with_contour, (line_center_x, roi_height // 2), 5, (0, 0, 255), -1)
    else:
        print("Lỗi: Momen m00 bằng 0, không thể tính trọng tâm.")

    cv2.imshow(f"{window_name_prefix} Step 7 - Result (Ket qua cuoi cung)", roi_with_contour)
    print(f"KẾT QUẢ CUỐI CÙNG cho {window_name_prefix}: {line_center_x}")
    return line_center_x

# ==============================================================================
# PHẦN 3: HÀM MAIN ĐỂ CHẠY GỠ LỖI
# ==============================================================================

if __name__ == '__main__':
    # THAY TÊN FILE ẢNH VÀO ĐÂY
    image_path = "test.jpg"
    
    try:
        original_image = cv2.imread(image_path)
        if original_image is None:
            raise FileNotFoundError(f"Không thể tải ảnh từ '{image_path}'. Hãy chắc chắn file tồn tại.")
        
        # Thay đổi kích thước ảnh về đúng size mà robot xử lý
        resized_image = cv2.resize(original_image, (WIDTH, HEIGHT))
        cv2.imshow("Original Resized Image", resized_image)

        # --- Chạy mô phỏng cho ROI Chính (Execution ROI) ---
        visualize_get_line_center(
            resized_image.copy(), 
            ROI_Y, 
            ROI_H, 
            window_name_prefix="ROI Chinh"
        )

        # --- Chạy mô phỏng cho ROI Dự báo (Lookahead ROI) ---
        visualize_get_line_center(
            resized_image.copy(), 
            LOOKAHEAD_ROI_Y, 
            LOOKAHEAD_ROI_H, 
            window_name_prefix="ROI Du Bao"
        )
        
        print("\n=============================================")
        print("Đã hiển thị tất cả các bước. Nhấn phím bất kỳ trên một cửa sổ ảnh để thoát.")
        cv2.waitKey(0) # Đợi người dùng nhấn phím
        cv2.destroyAllWindows() # Đóng tất cả cửa sổ

    except Exception as e:
        print(f"Đã xảy ra lỗi: {e}")