# Line Angle Analysis - JetBot Navigation

## Tổng quan
Hệ thống phân tích góc line kết hợp dữ liệu từ Camera ROS và LiDAR để đưa ra thông tin chính xác về hướng di chuyển của robot.

## Các file chính

### 1. `line_angle_analyzer.py`
**Chạy độc lập để phân tích góc line:**
```bash
./run_line_angle_analyzer.sh
```

**Tính năng:**
- Phân tích góc line từ camera (dựa trên vị trí trọng tâm line)
- Phân tích góc line từ LiDAR (dựa trên clusters có đặc điểm line)
- So sánh và kết hợp hai nguồn dữ liệu
- Đưa ra khuyến nghị về hành động điều chỉnh

### 2. `ros_lidar_follower.py` (Đã tích hợp)
**Chạy robot với phân tích góc tích hợp:**
```bash
python3 ros_lidar_follower.py
```

**Tính năng mới:**
- Tự động phân tích góc line mỗi 2 giây khi robot đang bám line
- In ra thông tin so sánh giữa camera và LiDAR
- Không ảnh hưởng đến logic điều khiển hiện tại

## Thông tin đầu ra

### 📷 CAMERA Analysis
- **Angle**: Góc lệch của line so với trung tâm robot (degrees)
  - `Dương (+)`: Line ở bên phải robot
  - `Âm (-)`: Line ở bên trái robot
- **Confidence**: HIGH/MEDIUM/LOW dựa trên độ lệch
- **Direction**: LEFT/RIGHT/STRAIGHT

### 📡 LIDAR Analysis  
- **Angle**: Góc trung bình từ các clusters phát hiện được
- **Clusters**: Số lượng clusters line tìm thấy
- **Chi tiết clusters**: Số điểm, góc, khoảng cách trung bình

### 🔄 COMPARISON
- **Difference**: Độ chênh lệch góc giữa camera và LiDAR
- **Agreement**: EXCELLENT/GOOD/FAIR/POOR
- **Combined**: Góc kết hợp hoặc khuyến nghị sử dụng sensor nào

## Cách hoạt động

### Camera Line Detection
1. Tạo ROI (Region of Interest) ở phía dưới ảnh
2. Lọc màu sắc line (HSV thresholds)  
3. Tạo focus mask chỉ xét khu vực trung tâm
4. Tìm contour lớn nhất và tính trọng tâm
5. Chuyển đổi vị trí pixel thành góc

### LiDAR Line Detection
1. Chia dữ liệu LiDAR thành các zone 10°
2. Chỉ xét các zone ở phía trước (±45°)
3. Tìm clusters có đặc điểm của line:
   - Khoảng cách 0.15m - 2.0m
   - Ít nhất 5 điểm liên tiếp
   - Variance khoảng cách thấp (≤0.4)
4. Tính weighted average dựa trên số điểm và khoảng cách

### Kết hợp dữ liệu
- **HIGH confidence**: Cả hai sensor đều cho kết quả tin cậy → Trung bình
- **Một sensor HIGH**: Ưu tiên sensor có confidence cao
- **Cả hai MEDIUM/LOW**: Trung bình với cảnh báo

## Tham số có thể điều chỉnh

### Trong `ros_lidar_follower.py`:
```python
self.angle_analysis_enabled = True        # Bật/tắt phân tích
self.angle_analysis_interval = 2.0        # Tần suất phân tích (giây)
```

### Trong `line_angle_analyzer.py`:
```python
self.CAMERA_FOV = 60                      # Field of view camera
self.LINE_FRONT_ANGLE_RANGE = 45          # Phạm vi góc LiDAR
self.LINE_DISTANCE_VARIANCE_THRESHOLD = 0.3  # Ngưỡng variance cho line
self.min_distance = 0.15                  # Khoảng cách LiDAR tối thiểu
self.max_distance = 1.5                   # Khoảng cách LiDAR tối đa
```

## Debug và Troubleshooting

### Nếu không thấy dữ liệu:
1. Kiểm tra ROS topics:
   ```bash
   rostopic list
   rostopic echo /scan
   rostopic echo /csi_cam_0/image_raw
   ```

2. Kiểm tra tham số line detection:
   - Điều chỉnh `LINE_COLOR_LOWER/UPPER` cho line color
   - Điều chỉnh `SCAN_PIXEL_THRESHOLD` cho kích thước line tối thiểu

### Nếu góc camera và LiDAR chênh lệch nhiều:
1. Kiểm tra calibration camera
2. Điều chỉnh `CAMERA_FOV`
3. Kiểm tra mounting position của sensors

## Ví dụ Output

```
============================================================
🔍 LINE ANGLE ANALYSIS
⏰ Time: 123.4s
============================================================
📷 CAMERA:
   • Angle:  +3.25° (HIGH)
   • Direction: STRAIGHT (centered)
📡 LIDAR:
   • Angle:  +2.80° (MEDIUM)  
   • Clusters: 2 found
     #1: 8 pts @ +2.1°, dist=0.45m
     #2: 6 pts @ +3.5°, dist=0.62m
🔄 COMPARISON:
   • Difference: 0.45°
   • Agreement: EXCELLENT
   • Combined:  +3.03° (averaged)
============================================================
```

## Lưu ý
- Hệ thống chỉ hoạt động khi robot ở trạng thái `DRIVING_STRAIGHT`
- Phân tích không ảnh hưởng đến logic điều khiển robot hiện tại
- Có thể tắt bằng cách đặt `angle_analysis_enabled = False`