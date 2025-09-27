# LiDAR Checker Tool

## 📋 Mô tả
File `lidar_checker.py` là một công cụ độc lập để kiểm tra và phân tích dữ liệu LiDAR từ ROS topic `/scan`. Công cụ này giúp:

- ✅ Kiểm tra kết nối LiDAR
- 📊 Phân tích chất lượng dữ liệu
- 🎯 Phát hiện obstacles và clusters
- ⬆️  Phân tích khu vực phía trước (navigation)
- 📈 Đánh giá hiệu suất sensor

## 🚀 Cách sử dụng

### 1. Kiểm tra một lần (Single Check)
```bash
# Cách 1: Sử dụng script
./run_lidar_checker.sh

# Cách 2: Chạy trực tiếp
python3 lidar_checker.py
```

### 2. Monitoring liên tục (Continuous Mode)
```bash
# Monitoring mỗi 2 giây (mặc định)
./run_lidar_checker.sh --continuous

# Monitoring mỗi 1 giây
./run_lidar_checker.sh --continuous --interval=1

# Chạy trực tiếp
python3 lidar_checker.py --continuous --interval=2
```

### 3. Xem help
```bash
./run_lidar_checker.sh --help
```

## 📊 Thông tin phân tích

### Basic Info
- Timestamp và frame ID
- Số lượng điểm đo
- Phạm vi góc và khoảng cách
- Thời gian scan

### Range Analysis  
- Thống kê khoảng cách (min, max, mean, median, std)
- Tỷ lệ điểm hợp lệ vs không hợp lệ
- Phân loại điểm đo

### Front Sector Analysis (±45°)
- Phân tích khu vực phía trước quan trọng cho navigation
- Phát hiện obstacle gần nhất
- Phân loại khoảng cách: very close, close, medium, far
- Đưa ra cảnh báo navigation

### Object Detection
- Tìm các object trong detection range (0.15m - 2.0m)
- Nhóm điểm thành clusters
- Thông tin chi tiết từng cluster

### Scan Quality
- Đánh giá chất lượng tổng thể
- Thống kê các loại reading (valid, inf, nan, etc.)
- Xếp hạng: EXCELLENT, GOOD, FAIR, POOR, VERY POOR

## ⚙️ Tham số cấu hình

Trong file `lidar_checker.py`:

```python
self.min_distance = 0.15        # Khoảng cách tối thiểu detect object
self.max_distance = 2.0         # Khoảng cách tối đa detect object  
self.front_angle_range = 45     # Phạm vi góc phía trước (±45°)
```

## 🔧 Troubleshooting

### Lỗi thường gặp:

1. **"roscore is not running"**
   ```bash
   # Terminal 1
   roscore
   
   # Terminal 2  
   ./run_lidar_checker.sh
   ```

2. **"/scan topic NOT found"**
   - Kiểm tra LiDAR node có chạy không
   - Kiểm tra topic name có đúng không: `rostopic list`

3. **"No data received"**
   - LiDAR có kết nối không
   - Kiểm tra permissions USB (nếu dùng USB LiDAR)
   - Kiểm tra topic có publishing không: `rostopic hz /scan`

4. **"Timeout: No LiDAR data"**
   - LiDAR node chưa start
   - Network issue (nếu remote)
   - Hardware problem

## 📖 Ví dụ Output

```
================================================================================
🔍 COMPLETE LIDAR ANALYSIS
   Time: 15.3s since start
   Scan count: 42
================================================================================

📊 LIDAR BASIC INFO
============================================================
   • Timestamp: 1758980682.257090
   • Frame ID: laser_frame
   • Total points: 360
   • Angle range: -180.0° to 180.0°
   • Angle increment: 1.000°
   • Distance range: 0.120m to 12.000m
   • Scan duration: 0.100s

📏 RANGE ANALYSIS:
   • Valid points: 340/360 (94.4%)
   • Invalid/Inf points: 20
   • Min distance: 0.165m
   • Max distance: 8.432m
   • Mean distance: 2.341m
   • Median distance: 1.876m
   • Std deviation: 1.234m

⬆️  FRONT SECTOR ANALYSIS (±45°):
   • Valid front points: 78
   • Min front distance: 0.234m
   • Max front distance: 3.456m
   • Avg front distance: 1.234m
   • Closest obstacle: 0.234m at 12.3°
   • Very close (<0.3m): 3 points
   • Close (0.3-0.8m): 15 points  
   • Medium (0.8-1.5m): 25 points
   • Far (>1.5m): 35 points
   • ✅ Path ahead looks clear

🎯 OBJECTS IN DETECTION RANGE (0.15m - 2.0m):
   • Total points in range: 89
   • Detected clusters: 3
     Cluster 1: 12 points
       • Center: -34.5°
       • Avg distance: 0.456m
       • Angle span: 8.7°
       • Range: -38.9° to -30.1°
     Cluster 2: 8 points
       • Center: 15.2°
       • Avg distance: 1.234m
       • Angle span: 5.3°
       • Range: 12.6° to 17.9°

📈 SCAN QUALITY ANALYSIS:
   • Valid readings: 340/360 (94.4%)
   • Infinite readings: 15 (4.2%)
   • NaN readings: 5 (1.4%)
   • Too close readings: 0
   • Too far readings: 0
   • Overall quality: EXCELLENT (94.4%)
```

## 🔄 Integration với hệ thống khác

File này có thể được import và sử dụng trong các file khác:

```python
from lidar_checker import LidarChecker

# Khởi tạo
checker = LidarChecker()

# Đợi data
rospy.sleep(1.0)

# Phân tích
if checker.latest_scan is not None:
    checker.print_full_analysis()
```