### **Mô tả Kỹ thuật: Hệ thống Điều khiển Tự hành JetBot dựa trên Lộ trình và Sự kiện**

#### I. Yêu cầu và Mục tiêu Hệ thống

Hệ thống được thiết kế để đáp ứng một tập hợp các yêu cầu phức tạp, biến JetBot từ một robot dò line đơn giản thành một tác nhân tự hành thông minh có khả năng hoạt động trong một môi trường có cấu trúc và quy tắc.

**Yêu cầu Chức năng:**
1.  **Điều hướng theo Lộ trình:** Robot phải có khả năng đọc một bản đồ số (file `map.json`), tự động tìm đường đi ngắn nhất từ điểm bắt đầu đến điểm kết thúc bằng thuật toán A*.
2.  **Bám vạch Kẻ đường:** Robot phải tự động di chuyển theo một vạch kẻ màu đen trên nền trắng, tự động căn chỉnh để đi vào giữa vạch.
3.  **Phát hiện Giao lộ:** Robot phải nhận biết được khi nó đến một giao lộ (ngã ba, ngã tư) bằng cách sử dụng kết hợp cảm biến LiDAR và phân tích hình ảnh camera.
4.  **Nhận diện và Diễn giải Biển báo:** Sử dụng một mô hình Trí tuệ Nhân tạo (YOLOv8-ONNX), robot phải phát hiện và phân loại chính xác các biển báo giao thông, bao gồm:
    *   **Biển báo Bắt buộc:**
    *   **Biển báo Cấm:**
5.  **Ra quyết định Phức hợp:** Tại mỗi giao lộ, robot phải ra quyết định hướng đi dựa trên một hệ thống ưu tiên nghiêm ngặt, kết hợp giữa lộ trình đã định, biển báo và các quy tắc giao thông.
6.  **Thích ứng và Lập lại Kế hoạch:** Robot phải có khả năng tự động tìm một con đường mới nếu lộ trình ban đầu bị chặn bởi biển báo cấm, hoặc nếu nó buộc phải đi chệch hướng do một biển báo bắt buộc.
7.  **Thu thập Dữ liệu:** Robot phải có khả năng nhận diện các đối tượng dữ liệu như mã QR hoặc bài toán và gửi thông tin thu thập được qua giao thức MQTT.

**Yêu cầu Phi chức năng:**
1.  **Tính Tin cậy:** Hành vi của robot phải nhất quán và có thể dự đoán được, đặc biệt là tại các giao lộ và góc cua gấp.
2.  **Tính Mạnh mẽ (Robustness):** Hệ thống phải có khả năng chống chọi với các điều kiện thực tế không hoàn hảo như nền nhà bị bóng, ánh sáng thay đổi, và bóng đổ.
3.  **Kiến trúc Mô-đun:** Code phải được cấu trúc một cách rõ ràng, tách biệt các chức năng (điều khiển, điều hướng, cảm biến) để dễ bảo trì và mở rộng.

---

#### II. Kiến trúc Hệ thống

Để quản lý sự phức tạp của các yêu cầu, hệ thống được xây dựng dựa trên hai mẫu kiến trúc chính: **Máy Trạng thái Hữu hạn (Finite State Machine - FSM)** và **Thiết kế Mô-đun (Modular Design)**.

**1. Máy Trạng thái Hữu hạn (FSM):**
Đây là trái tim của hệ thống, quản lý hành vi của robot bằng cách chuyển nó qua các trạng thái được định nghĩa rõ ràng.

*   `DRIVING_STRAIGHT`: Trạng thái hoạt động chính. Robot tích cực bám line và sử dụng các cảm biến để theo dõi các sự kiện sắp xảy ra.
*   `APPROACHING_INTERSECTION`: Trạng thái "phòng ngừa". Khi camera dự báo có giao lộ ở phía trước, robot vào trạng thái này, đi thẳng một đoạn ngắn để đảm bảo nó dừng lại ở đúng trung tâm, thay vì phản ứng quá sớm.
*   `HANDLING_EVENT`: Trạng thái "xử lý". Robot đã dừng hẳn tại giao lộ. Mọi hoạt động phức tạp như quay quét biển báo, chạy AI, và ra quyết định diễn ra ở đây.
*   `LEAVING_INTERSECTION`: Trạng thái "ổn định sau quyết định". Robot đi thẳng một đoạn để thoát khỏi khu vực trung tâm giao lộ, tránh việc các cảm biến bị nhiễu bởi chính giao lộ nó vừa xử lý.
*   `REACQUIRING_LINE`: Trạng thái "tái định vị". Sau khi rời giao lộ, robot đi thẳng và chủ động tìm kiếm vạch kẻ của con đường mới. Trạng thái này cực kỳ quan trọng để ngăn lỗi phát hiện giao lộ giả ngay sau khi rẽ.
*   `DEAD_END` / `GOAL_REACHED`: Các trạng thái kết thúc.

**2. Thiết kế Mô-đun:**
Các chức năng chính được tách thành các thành phần độc lập:

*   `JetBotController`: Lớp điều khiển chính, hoạt động như "bộ não" quản lý FSM và tích hợp tất cả các mô-đun khác.
*   `MapNavigator`: Lớp điều hướng, chịu trách nhiệm về mọi thứ liên quan đến bản đồ: đọc file, xây dựng đồ thị, và cung cấp các truy vấn tìm đường (A*).
*   `SimpleOppositeDetector`: Lớp cảm biến, trừu tượng hóa việc xử lý dữ liệu thô từ LiDAR để đưa ra một tín hiệu đơn giản: "có giao lộ" hoặc "không có".
*   **Hệ thống Thị giác Máy tính:** Bao gồm các hàm xử lý ảnh (`_get_line_center`) và mô hình YOLO để nhận diện đối tượng.

---

#### III. Chi tiết Kỹ thuật và Cách thức Triển khai

Để giải quyết các thách thức cụ thể, hệ thống đã triển khai các kỹ thuật tiên tiến:

**1. Kỹ thuật "Nhìn Xa Hơn" (Lookahead ROI) để xử lý góc cua T/L:**
*   **Vấn đề:** Khi đến góc cua gấp, logic bám line phản ứng quá nhanh, khiến robot rẽ trước khi kịp nhận ra đó là một giao lộ cần dừng lại.
*   **Triển khai:**
    *   Sử dụng **hai Vùng Quan Tâm (ROI)** trên camera.
    *   **ROI Dự báo (`LOOKAHEAD_ROI`):** Đặt ở vị trí cao hơn trong ảnh (nhìn xa hơn). Nhiệm vụ của nó là phát hiện sự "biến mất" của vạch kẻ ở phía xa.
    *   **ROI Thực thi (`EXECUTION_ROI`):** Đặt ở gần robot, dùng để bám line.
    *   **Logic ưu tiên:** Trong vòng lặp chính, hệ thống **luôn kiểm tra ROI Dự báo trước**. Nếu phát hiện vạch kẻ sắp biến mất, nó sẽ ngay lập tức kích hoạt trạng thái `APPROACHING_INTERSECTION` và **bỏ qua hoàn toàn** dữ liệu từ ROI Thực thi. Điều này ngăn chặn triệt để hành vi rẽ sớm không mong muốn.

**2. Kỹ thuật "Mặt nạ Tập trung" (Focus Mask) để xử lý ngã tư:**
*   **Vấn đề:** Khi dò line đen ở ngã tư, trọng tâm của toàn bộ khối đen hình chữ thập bị tính toán sai, kéo robot lệch về một bên và gây ra các cú rẽ giả.
*   **Triển khai:**
    *   Trong hàm `_get_line_center`, sau khi tạo mặt nạ màu đen (`color_mask`), một **mặt nạ thứ hai** (`focus_mask`) được tạo ra.
    *   `focus_mask` là một hình chữ nhật trắng hẹp ở chính giữa, hoạt động như một "cặp kính che".
    *   Sử dụng phép toán `cv2.bitwise_and`, hệ thống chỉ giữ lại những pixel nào thuộc cả hai mặt nạ.
    *   **Kết quả:** Các phần vạch kẻ ngang của giao lộ bị loại bỏ khỏi tính toán. Robot chỉ "nhìn thấy" vạch kẻ đi thẳng và giữ được hướng đi chính xác.

**3. Kỹ thuật "Giới hạn Điều chỉnh" và "Thu hẹp Vùng An Toàn":**
*   **Vấn đề 1:** Sau khi dùng Focus Mask, robot trở nên "lười" căn giữa.
*   **Vấn đề 2:** Vẫn cần một cơ chế an toàn để tránh các cú bẻ lái quá gắt.
*   **Triển khai:**
    *   **Thu hẹp `SAFE_ZONE_PERCENT`:** Vùng an toàn (nơi robot chỉ đi thẳng) được thu nhỏ lại đáng kể. Điều này buộc robot phải tích cực thực hiện các điều chỉnh nhỏ để giữ mình ở chính giữa, thay vì hài lòng với việc đi lệch.
    *   **Giới hạn `MAX_CORRECTION_ADJ`:** Trong hàm `correct_course`, lực điều chỉnh (`adj`) được tính toán và sau đó bị "kẹp" lại bởi hàm `np.clip`. Điều này đảm bảo rằng dù sai số có lớn đến đâu, robot cũng không bao giờ gửi một lệnh chênh lệch tốc độ giữa hai bánh xe vượt quá một ngưỡng an toàn, giúp robot cua mượt mà thay vì giật cục.

**4. Logic Lập lại Kế hoạch và Tuân thủ Quy tắc:**
*   **Triển khai:**
    *   Hệ thống duy trì một danh sách các "cạnh đường bị cấm" (`banned_edges`).
    *   Tại giao lộ, logic trong `handle_intersection` tuân theo một cây quyết định nghiêm ngặt:
        1.  Có biển báo bắt buộc không? Nếu có, `intended_action` được xác định.
        2.  Nếu không, `intended_action` được lấy từ kế hoạch A*.
        3.  Kiểm tra xem `intended_action` có vi phạm biển báo cấm không.
        4.  **Nếu có vi phạm và hành động đến từ kế hoạch A*:** Thêm cạnh tương ứng vào `banned_edges` và tìm lại đường đi mới.
        5.  **Nếu hành động đến từ biển báo bắt buộc và buộc robot đi chệch kế hoạch:** Robot sẽ tuân thủ, sau đó xác định node mới nó đã đến và **tính toán lại toàn bộ lộ trình** từ vị trí mới đó.


### **Luồng Hoạt động Chi tiết của Hệ thống**

#### Giai đoạn 1: Khởi tạo và Chuẩn bị

1.  **Khởi động Node ROS:** Hệ thống được khởi chạy như một node ROS.
2.  **Tải Tham số (`setup_parameters`):** Tất cả các hằng số vận hành được nạp vào bộ nhớ: tốc độ, thông số ROI (bao gồm cả ROI Dự báo), dải màu HSV cho vạch đen, ngưỡng tin cậy của YOLO, v.v.
3.  **Khởi tạo Phần cứng và AI (`initialize_...`):**
    *   Kết nối và kiểm soát động cơ JetBot.
    *   Tải mô hình YOLOv8 từ file `.onnx` vào ONNX Runtime, sẵn sàng cho việc nhận diện.
    *   Thiết lập kết nối đến MQTT Broker.
4.  **Lập Kế hoạch Lộ trình Ban đầu (`plan_initial_route`):**
    *   Đối tượng `MapNavigator` được tạo, đọc file `map.json` và xây dựng một đồ thị biểu diễn bản đồ.
    *   Thuật toán A* được gọi để tìm đường đi ngắn nhất từ `start_node` đến `end_node`. Ví dụ, kết quả là `planned_path = [1, 2, 5, 8]`.
    *   Hệ thống xác định trạng thái ban đầu:
        *   `current_node_id` = `1` (node đầu tiên trong path).
        *   `target_node_id` = `2` (node thứ hai trong path).
5.  **Vào Trạng thái Hoạt động:** Robot chuyển sang trạng thái `DRIVING_STRAIGHT` và bắt đầu vòng lặp chính.

---

#### Giai đoạn 2: Vòng lặp Bám line và Dự báo Sự kiện (Trạng thái `DRIVING_STRAIGHT`)

*Robot đang di chuyển trên đoạn đường nối `node 1` và `node 2`.*

1.  **Thu thập Dữ liệu Cảm biến:** Trong mỗi vòng lặp, hệ thống nhận dữ liệu mới nhất từ topic camera (`latest_image`) và LiDAR (`detector`).
2.  **Kiểm tra Ưu tiên 1 (LiDAR):** Hệ thống hỏi `detector`: "Có thấy không gian mở đặc trưng của giao lộ không?".
    *   **Nếu CÓ:** Đây là tín hiệu chắc chắn nhất. Luồng hoạt động chuyển sang **Giai đoạn 3**.
    *   **Nếu KHÔNG:** Tiếp tục bước tiếp theo.
3.  **Kiểm tra Ưu tiên 2 (ROI Dự báo):** Hệ thống gọi `_get_line_center` với các tham số của **ROI Dự báo** (ở phía xa).
    *   **Nếu kết quả là `None` (không thấy line):** Hệ thống kết luận: "Cảnh báo! Sắp đến một giao lộ/điểm kết thúc." Luồng hoạt động chuyển sang **Giai đoạn 3**.
    *   **Nếu kết quả CÓ giá trị:** Phía trước vẫn an toàn. Tiếp tục bước tiếp theo.
4.  **Thực thi Bám line (Ưu tiên 3):**
    *   Hệ thống gọi `_get_line_center` với các tham số của **ROI Chính** (ở gần).
    *   Kết quả `line_center_x` được đưa vào hàm `correct_course`.
    *   `correct_course` tính toán `error`, áp dụng các cơ chế **Giới hạn Điều chỉnh** (`np.clip`) và **Vùng An Toàn Thu hẹp** để tính toán và gửi lệnh tốc độ phù hợp cho hai động cơ. Robot tiếp tục đi và tự căn giữa.
5.  Vòng lặp này tiếp tục với tần suất cao (ví dụ 20Hz), liên tục điều chỉnh và kiểm tra.

---

#### Giai đoạn 3: Xử lý Giao lộ (Chuỗi các Trạng thái)

*Giả sử robot sắp đến `node 2`. Một trong các điều kiện ở Giai đoạn 2 được kích hoạt.*

1.  **Phát hiện và Chuyển trạng thái:**
    *   **Nếu do LiDAR phát hiện:** Robot **dừng ngay lập tức**. Trạng thái được cập nhật, `current_node_id` được gán bằng `target_node_id` (giờ là `2`). Hệ thống gọi `handle_intersection`.
    *   **Nếu do ROI Dự báo phát hiện:** Robot chuyển sang trạng thái `APPROACHING_INTERSECTION`.

2.  **Tiến vào Giao lộ (Trạng thái `APPROACHING_INTERSECTION`, nếu cần):**
    *   Robot bỏ qua mọi cảm biến và chỉ đi thẳng với `BASE_SPEED` trong một khoảng thời gian ngắn (`INTERSECTION_APPROACH_DURATION`, ví dụ 0.5s).
    *   Hết thời gian, robot **dừng hẳn**.
    *   Trạng thái được cập nhật, `current_node_id` được gán bằng `target_node_id` (giờ là `2`). Hệ thống gọi `handle_intersection`.

3.  **Suy nghĩ và Ra quyết định (Hàm `handle_intersection`):**
    *   Robot bây giờ đang đứng yên tại `node 2`.
    *   **Quét Biển báo:** Robot thực hiện một chuỗi hành động quay: quay 45 độ, chụp ảnh, chạy `detect_with_yolo`, quay về vị trí cũ.
    *   **Lấy Kế hoạch:** Hỏi `MapNavigator`: "Từ `node 2`, theo kế hoạch A* thì nên đi đến node nào?". Giả sử câu trả lời là `node 5`.
    *   **Cây Quyết định:**
        *   Có biển báo bắt buộc nào không? (Ví dụ: "Rẽ Trái")
        *   Nếu có -> `intended_action` = "rẽ trái". Robot nhận ra đây là một sự **chệch hướng** so với kế hoạch (đi đến `node 5`).
        *   Nếu không -> `intended_action` = hành động tương ứng để đi đến `node 5`.
        *   Kiểm tra `intended_action` với các biển báo cấm.
        *   Nếu bị cấm -> Thêm cạnh đường này vào `banned_edges`, tìm lại đường đi mới, và lặp lại cây quyết định.
    *   **Thực thi Hành động:** Robot thực hiện hành động cuối cùng (`final_decision`). Ví dụ: `self.turn_robot(-90, True)` để rẽ trái.
    *   **Đặt Mục tiêu Mới:**
        *   Nếu bị chệch hướng, robot xác định node mới sau khi rẽ trái (ví dụ là `node 4`) và tính toán lại toàn bộ `planned_path` từ `node 4` về đích (`node 8`).
        *   Hệ thống đặt `target_node_id` mới (ví dụ là `node 6`, node tiếp theo trong lộ trình mới).
        *   Nếu không chệch hướng, chỉ cần đặt `target_node_id` là node tiếp theo trong kế hoạch cũ (`node 5`).
    *   **Chuyển trạng thái:** Cuối cùng, robot chuyển sang trạng thái `LEAVING_INTERSECTION`.

4.  **Rời khỏi Giao lộ (Trạng thái `LEAVING_INTERSECTION`):**
    *   Robot đi thẳng với `BASE_SPEED` trong `INTERSECTION_CLEARANCE_DURATION` (ví dụ 1.5s) để đảm bảo nó đã hoàn toàn thoát khỏi khu vực trung tâm của `node 2`.
    *   Hết thời gian, nó chuyển sang `REACQUIRING_LINE`.

5.  **Tìm lại Line (Trạng thái `REACQUIRING_LINE`):**
    *   Robot tiếp tục đi thẳng.
    *   Trong mỗi vòng lặp, nó gọi `_get_line_center` (chỉ với ROI Chính) để tìm vạch kẻ của con đường mới (đường nối đến `target_node_id` mới).
    *   **Khi tìm thấy:** Chuyển về trạng thái `DRIVING_STRAIGHT`.
    *   **Nếu hết timeout** mà không tìm thấy -> Chuyển sang `DEAD_END`.

---

#### Giai đoạn 4: Hoàn thành và Kết thúc

*   Chu trình từ Giai đoạn 2 đến 3 lặp đi lặp lại.
*   Khi robot đến một giao lộ và cập nhật `current_node_id` thành `8` (chính là `end_node`), thay vì gọi `handle_intersection`, hệ thống sẽ chuyển thẳng sang trạng thái `GOAL_REACHED`.
*   Robot dừng lại vĩnh viễn, phát thông báo hoàn thành nhiệm vụ và giải phóng tài nguyên.