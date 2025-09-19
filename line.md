Hệ thống dò line hiện tại có thể được mô tả như một quy trình xử lý ảnh đa tầng, được thiết kế để vừa **chính xác**, vừa **mạnh mẽ (robust)** và vừa **thông minh** trong bối cảnh cụ thể của robot.

---

### **Giải thích Chi tiết Hệ thống Dò Line**

#### I. Nguyên tắc cốt lõi: Phân tích màu sắc trong Không gian màu HSV

Thay vì sử dụng không gian màu RGB (Red, Green, Blue) thông thường, hệ thống chuyển đổi hình ảnh sang không gian màu **HSV (Hue, Saturation, Value)**.

*   **Tại sao lại dùng HSV?**
    *   **Hue (H - Tông màu):** Đại diện cho màu sắc thực tế (đỏ, xanh, vàng...).
    *   **Saturation (S - Độ bão hòa):** Đại diện cho "độ đậm" của màu. Màu xám có S thấp, màu rực rỡ có S cao.
    *   **Value (V - Độ sáng):** Đại diện cho độ sáng/tối của màu.
    *   Ưu điểm lớn nhất của HSV là nó **tách biệt thông tin độ sáng (V) ra khỏi thông tin màu sắc (H, S)**. Điều này cực kỳ hữu ích trong môi trường có ánh sáng không đồng đều. Một vạch kẻ đen có thể có các sắc thái đen khác nhau do bị đèn chiếu vào (chỗ bóng, chỗ sáng hơn), nhưng điểm chung của chúng là đều có thành phần **Value (V) rất thấp**.

*   **Cách triển khai:**
    *   Trong `setup_parameters`, chúng ta định nghĩa một dải màu cho vạch đen:
        ```python
        self.LINE_COLOR_LOWER = np.array([0, 0, 0])
        self.LINE_COLOR_UPPER = np.array([180, 255, 60])
        ```
    *   **Diễn giải:** "Hãy tìm cho tôi tất cả các pixel có:
        *   Tông màu `H` bất kỳ (từ 0 đến 180).
        *   Độ bão hòa `S` bất kỳ (từ 0 đến 255).
        *   Nhưng độ sáng `V` phải cực kỳ thấp (từ 0 đến 60)."
    *   Hàm `cv2.inRange(hsv, lower, upper)` sẽ thực hiện việc này, tạo ra một "mặt nạ" (mask) - một ảnh nhị phân (đen trắng), trong đó các pixel màu trắng đại diện cho những vùng được xác định là vạch kẻ.

---

#### II. Kỹ thuật "Nhìn Xa Hơn" (Lookahead ROI): Tránh phản ứng sai tại giao lộ

Đây là lớp phòng thủ thông minh đầu tiên của hệ thống.

*   **Vấn đề cần giải quyết:** Robot rẽ quá sớm tại các góc cua chữ T/L vì logic bám line phản ứng nhanh hơn logic nhận biết đó là một giao lộ.
*   **Triển khai:**
    *   Hệ thống không chỉ nhìn xuống "chân" mà còn "ngước nhìn" xa hơn về phía trước. Điều này được thực hiện bằng cách định nghĩa **hai Vùng Quan Tâm (ROI)**:
        1.  **ROI Dự báo (`LOOKAHEAD_ROI`):** Một hình chữ nhật ảo được đặt ở nửa trên của khung hình camera.
        2.  **ROI Thực thi (`EXECUTION_ROI`):** Một hình chữ nhật ảo đặt ở dưới cùng của khung hình.
    *   **Luồng logic ưu tiên trong `run()`:**
        1.  **Kiểm tra Dự báo trước:** Hệ thống **luôn** phân tích ROI Dự báo đầu tiên. Nó hỏi: "Ở phía xa có còn thấy vạch kẻ không?".
        2.  **Hành động Phòng ngừa:** Nếu câu trả lời là "Không", hệ thống ngay lập tức kết luận rằng sắp có một sự kiện (giao lộ/cuối đường). Nó sẽ **chủ động** chuyển sang trạng thái `APPROACHING_INTERSECTION` và **bỏ qua hoàn toàn** việc phân tích ROI Thực thi.
        3.  **Hành động Bình thường:** Chỉ khi ROI Dự báo báo cáo "an toàn" (vẫn thấy vạch kẻ), hệ thống mới tiếp tục phân tích ROI Thực thi để thực hiện việc bám line.
*   **Kết quả:** Kỹ thuật này hoạt động như một hệ thống cảnh báo sớm, cho phép robot chuẩn bị cho giao lộ trước khi nó đến quá gần và phạm sai lầm.

---

#### III. Kỹ thuật "Mặt nạ Tập trung" (Focus Mask): Giữ vững hướng đi tại ngã tư

Đây là lớp phòng thủ thứ hai, giải quyết vấn đề nhận thức khi robot đã ở trong một giao lộ phức tạp.

*   **Vấn đề cần giải quyết:** Tại ngã tư, vạch kẻ đen tạo thành một hình chữ thập lớn. Việc tính toán "trọng tâm" của cả khối đen này sẽ cho ra một kết quả sai, bị kéo lệch về phía các nhánh rẽ, khiến robot rẽ nhầm thay vì đi thẳng.
*   **Triển khai:**
    *   Trong hàm `_get_line_center`, sau khi đã tạo ra mặt nạ màu sắc (`color_mask`) từ ảnh HSV, hệ thống không sử dụng nó ngay.
    *   Nó tạo ra một **mặt nạ thứ hai** một cách linh động, gọi là `focus_mask`. Mặt nạ này chỉ đơn giản là một hình chữ nhật màu trắng hẹp nằm ở chính giữa của ROI (ví dụ, chỉ chiếm 50% chiều rộng trung tâm).
    *   Sử dụng phép toán `cv2.bitwise_and`, hệ thống kết hợp hai mặt nạ. Phép toán này hoạt động như một bộ lọc: "Chỉ giữ lại những pixel nào **vừa thuộc vạch kẻ đen VÀ vừa nằm trong khu vực tập trung**".
    *   Tất cả các phần của vạch kẻ ngang nằm ngoài khu vực trung tâm sẽ bị "xóa" khỏi mặt nạ cuối cùng.
*   **Kết quả:** Khi tính toán trọng tâm trên mặt nạ cuối cùng, robot chỉ còn "nhìn thấy" phần vạch kẻ đi thẳng. Nó không còn bị "phân tâm" bởi các lối rẽ, cho phép nó duy trì hướng đi thẳng một cách ổn định qua ngã tư.

---

#### IV. Tính toán vị trí và Điều khiển Động cơ (`correct_course`)

Sau khi đã có một "mặt nạ" sạch sẽ và đáng tin cậy, các bước cuối cùng được thực hiện:

1.  **Tìm Contour (`cv2.findContours`):** Tìm tất cả các vùng trắng (vùng vạch kẻ) liền mạch trên mặt nạ.
2.  **Xác định Vạch kẻ chính:** Chọn contour có diện tích lớn nhất, giả định rằng đó là vạch kẻ đường chính. Một ngưỡng diện tích tối thiểu (`SCAN_PIXEL_THRESHOLD`) được áp dụng để loại bỏ các đốm nhiễu nhỏ.
3.  **Tính Trọng tâm (`cv2.moments`):** Tính toán tọa độ `(x, y)` của trọng tâm hình học của contour vạch kẻ chính. Chúng ta chỉ quan tâm đến giá trị `x` (`line_center_x`).
4.  **Tính toán Sai số (`error`):** `error = line_center_x - (WIDTH / 2)`.
    *   `error > 0`: Vạch kẻ ở bên phải trung tâm.
    *   `error < 0`: Vạch kẻ ở bên trái trung tâm.
    *   `error = 0`: Robot ở chính giữa.
5.  **Điều khiển PID (Tỷ lệ - Proportional):**
    *   Hệ thống sử dụng một bộ điều khiển tỷ lệ đơn giản để điều chỉnh tốc độ động cơ.
    *   Một "lực điều chỉnh" (`adj`) được tính toán, tỷ lệ thuận với `error` và một hệ số khuếch đại (`CORRECTION_GAIN`).
    *   **Cơ chế An toàn:** `adj` được giới hạn bởi `np.clip` để đảm bảo nó không bao giờ vượt quá `MAX_CORRECTION_ADJ`. Điều này ngăn chặn các cú bẻ lái quá gắt, giúp robot cua mượt hơn.
    *   **Lệnh cuối cùng:** `robot.set_motors(BASE_SPEED + adj, BASE_SPEED - adj)`. Nếu vạch kẻ ở bên phải (`adj > 0`), bánh trái sẽ quay nhanh hơn và bánh phải quay chậm hơn, khiến robot rẽ sang phải để căn chỉnh lại.

Bằng cách kết hợp nhiều lớp xử lý và các kỹ thuật phòng ngừa, hệ thống dò line này không chỉ đơn thuần là "tìm một màu", mà đã trở thành một hệ thống nhận thức tinh vi, có khả năng dự báo, tập trung và hành động một cách an toàn trong các điều kiện phức tạp.