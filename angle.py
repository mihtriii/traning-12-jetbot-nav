#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
import math
from sensor_msgs.msg import LaserScan, Image

class AngleChecker:
    def __init__(self):
        """
        Khởi tạo AngleChecker để kiểm tra góc giữa robot và line.
        """
        rospy.loginfo("Khởi tạo AngleChecker...")
        
        # Camera parameters
        self.WIDTH, self.HEIGHT = 300, 300
        self.ROI_Y = int(self.HEIGHT * 0.85)
        self.ROI_H = int(self.HEIGHT * 0.15)
        self.CAMERA_FOV = 60  # Field of view của camera (degrees)
        
        # Line detection parameters
        self.LINE_COLOR_LOWER = np.array([0, 0, 0])
        self.LINE_COLOR_UPPER = np.array([180, 255, 75])
        self.SCAN_PIXEL_THRESHOLD = 100
        
        # LiDAR parameters
        self.LIDAR_FRONT_ANGLE_RANGE = 30  # ±30 degrees từ phía trước
        self.LIDAR_MIN_POINTS = 5  # Số điểm tối thiểu để phân tích
        self.LIDAR_LINE_DISTANCE_THRESHOLD = 1.0  # Khoảng cách để phát hiện "rãnh" của line
        
        # Trọng số kết hợp
        self.CAMERA_WEIGHT = 0.7
        self.LIDAR_WEIGHT = 0.3
        
        # Data storage
        self.latest_scan = None
        self.latest_image = None
        
        # ROS subscribers
        rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        rospy.Subscriber('/csi_cam_0/image_raw', Image, self.camera_callback)
        
        rospy.loginfo("AngleChecker đã sẵn sàng!")
    
    def scan_callback(self, scan_msg):
        """Callback để nhận dữ liệu LiDAR."""
        self.latest_scan = scan_msg
    
    def camera_callback(self, image_msg):
        """Callback để nhận dữ liệu camera."""
        try:
            if image_msg.encoding.endswith('compressed'):
                np_arr = np.frombuffer(image_msg.data, np.uint8)
                cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            else:
                cv_image = np.frombuffer(image_msg.data, dtype=np.uint8).reshape(
                    image_msg.height, image_msg.width, -1)
            
            if 'rgb' in image_msg.encoding:
                cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)
            
            self.latest_image = cv2.resize(cv_image, (self.WIDTH, self.HEIGHT))
        except Exception as e:
            rospy.logerr(f"Lỗi xử lý ảnh: {e}")
    
    def get_line_center_from_camera(self):
        """
        Tìm trọng tâm của line từ camera.
        Returns: vị trí x của line center, None nếu không tìm thấy
        """
        if self.latest_image is None:
            return None
        
        # Lấy ROI
        roi = self.latest_image[self.ROI_Y : self.ROI_Y + self.ROI_H, :]
        
        # Chuyển sang HSV và tạo mask
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.LINE_COLOR_LOWER, self.LINE_COLOR_UPPER)
        
        # Tìm contours
        _, contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if not contours:
            return None
        
        # Lấy contour lớn nhất
        largest_contour = max(contours, key=cv2.contourArea)
        
        if cv2.contourArea(largest_contour) < self.SCAN_PIXEL_THRESHOLD:
            return None
        
        # Tính trọng tâm
        M = cv2.moments(largest_contour)
        if M["m00"] > 0:
            return int(M["m10"] / M["m00"])
        
        return None
    
    def get_line_angle_from_camera(self):
        """
        Tính góc của line dựa trên vị trí trọng tâm từ camera.
        Returns: góc lệch (degrees), None nếu không xác định được
        """
        line_center = self.get_line_center_from_camera()
        if line_center is None:
            return None
        
        # Chuyển đổi vị trí pixel thành góc
        image_center = self.WIDTH / 2
        pixel_offset = line_center - image_center
        
        # Tính góc dựa trên FOV camera
        angle = (pixel_offset / image_center) * (self.CAMERA_FOV / 2)
        
        return angle
    
    def analyze_lidar_pattern(self):
        """
        Phân tích pattern từ dữ liệu LiDAR để tìm góc của line.
        Returns: góc lệch (degrees), None nếu không xác định được
        """
        if self.latest_scan is None:
            return None
        
        scan_data = self.latest_scan
        front_angles = []
        front_distances = []
        
        # Lọc các điểm LiDAR ở phía trước
        for i, distance in enumerate(scan_data.ranges):
            angle = scan_data.angle_min + i * scan_data.angle_increment
            angle_deg = math.degrees(angle)
            
            # Chỉ xét các điểm ở phía trước trong phạm vi cho phép
            if (-self.LIDAR_FRONT_ANGLE_RANGE <= angle_deg <= self.LIDAR_FRONT_ANGLE_RANGE and 
                scan_data.range_min < distance < scan_data.range_max):
                front_angles.append(angle_deg)
                front_distances.append(distance)
        
        if len(front_distances) < self.LIDAR_MIN_POINTS:
            rospy.logdebug("Không đủ điểm LiDAR để phân tích")
            return None
        
        # Tìm pattern của line (các điểm tạo ra "rãnh" trong dữ liệu)
        line_angles = []
        
        for i, distance in enumerate(front_distances):
            # Line thường tạo ra khoảng trống hoặc khoảng cách xa hơn
            if distance > self.LIDAR_LINE_DISTANCE_THRESHOLD:
                line_angles.append(front_angles[i])
        
        if len(line_angles) < 3:
            rospy.logdebug("Không tìm thấy đủ pattern của line trong LiDAR")
            return None
        
        # Tính góc trung bình của các điểm line
        avg_line_angle = np.mean(line_angles)
        
        # Lọc nhiễu bằng cách loại bỏ các điểm quá xa khỏi trung bình
        filtered_angles = [angle for angle in line_angles 
                          if abs(angle - avg_line_angle) < 10]  # ±10 degrees tolerance
        
        if len(filtered_angles) < 2:
            return avg_line_angle
        
        return np.mean(filtered_angles)
    
    def get_combined_line_angle(self):
        """
        Kết hợp thông tin từ camera và LiDAR để xác định góc chính xác nhất.
        Returns: góc kết hợp (degrees), None nếu không có thông tin
        """
        camera_angle = self.get_line_angle_from_camera()
        lidar_angle = self.analyze_lidar_pattern()
        
        rospy.loginfo(f"Camera angle: {camera_angle}, LiDAR angle: {lidar_angle}")
        
        # Kết hợp hai nguồn thông tin
        if camera_angle is not None and lidar_angle is not None:
            combined_angle = (self.CAMERA_WEIGHT * camera_angle + 
                            self.LIDAR_WEIGHT * lidar_angle)
            confidence = "HIGH"
        elif camera_angle is not None:
            combined_angle = camera_angle
            confidence = "MEDIUM"
        elif lidar_angle is not None:
            combined_angle = lidar_angle
            confidence = "LOW"
        else:
            return None, "NO_DATA"
        
        return combined_angle, confidence
    
    def is_line_trajectory_valid(self, max_allowed_angle=15):
        """
        Kiểm tra xem robot có đang đi đúng hướng line không.
        
        Args:
            max_allowed_angle: Góc tối đa cho phép (degrees)
            
        Returns:
            tuple: (is_valid, angle, confidence)
        """
        angle, confidence = self.get_combined_line_angle()
        
        if angle is None:
            return True, None, confidence  # Không đủ dữ liệu để phán đoán
        
        is_valid = abs(angle) <= max_allowed_angle
        
        if not is_valid:
            rospy.logwarn(f"Line angle quá lớn: {angle:.2f}°. Robot có thể đang đi sai hướng.")
        
        return is_valid, angle, confidence
    
    def get_correction_suggestion(self):
        """
        Đưa ra gợi ý điều chỉnh dựa trên góc line.
        
        Returns:
            dict: {'action': str, 'angle': float, 'confidence': str, 'description': str}
        """
        angle, confidence = self.get_combined_line_angle()
        
        if angle is None:
            return {
                'action': 'continue',
                'angle': 0,
                'confidence': confidence,
                'description': 'Không có dữ liệu góc, tiếp tục theo hướng hiện tại'
            }
        
        # Xác định hành động cần thiết
        if abs(angle) < 3:
            action = 'continue'
            description = 'Góc line tốt, tiếp tục đi thẳng'
        elif 3 <= abs(angle) < 10:
            action = 'slight_correction'
            description = f'Điều chỉnh nhẹ {"trái" if angle > 0 else "phải"}'
        elif 10 <= abs(angle) < 20:
            action = 'moderate_correction'
            description = f'Điều chỉnh vừa phải {"trái" if angle > 0 else "phải"}'
        else:
            action = 'strong_correction'
            description = f'Cần điều chỉnh mạnh {"trái" if angle > 0 else "phải"}'
        
        return {
            'action': action,
            'angle': angle,
            'confidence': confidence,
            'description': description
        }
    
    def visualize_angle_info(self):
        """
        Tạo ảnh visualization hiển thị thông tin góc.
        Returns: ảnh với thông tin góc được vẽ lên
        """
        if self.latest_image is None:
            return None
        
        vis_image = self.latest_image.copy()
        
        # Vẽ ROI
        cv2.rectangle(vis_image, (0, self.ROI_Y), 
                     (self.WIDTH-1, self.ROI_Y + self.ROI_H), (0, 255, 0), 2)
        
        # Vẽ line center nếu tìm thấy
        line_center = self.get_line_center_from_camera()
        if line_center is not None:
            cv2.line(vis_image, (line_center, self.ROI_Y), 
                    (line_center, self.ROI_Y + self.ROI_H), (0, 0, 255), 3)
        
        # Vẽ center line của image
        image_center = self.WIDTH // 2
        cv2.line(vis_image, (image_center, 0), 
                (image_center, self.HEIGHT), (255, 255, 0), 1)
        
        # Lấy thông tin góc
        angle, confidence = self.get_combined_line_angle()
        suggestion = self.get_correction_suggestion()
        
        # Vẽ text thông tin
        y_offset = 20
        if angle is not None:
            cv2.putText(vis_image, f"Angle: {angle:.1f}deg", 
                       (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            y_offset += 25
        
        cv2.putText(vis_image, f"Confidence: {confidence}", 
                   (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        y_offset += 25
        
        cv2.putText(vis_image, f"Action: {suggestion['action']}", 
                   (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        
        return vis_image

def main():
    """Test function để chạy AngleChecker độc lập."""
    rospy.init_node('angle_checker_node', anonymous=True)
    
    try:
        checker = AngleChecker()
        rate = rospy.Rate(10)  # 10 Hz
        
        rospy.loginfo("Bắt đầu kiểm tra góc. Nhấn Ctrl+C để dừng.")
        
        while not rospy.is_shutdown():
            # Kiểm tra tính hợp lệ của trajectory
            is_valid, angle, confidence = checker.is_line_trajectory_valid()
            
            # Lấy gợi ý điều chỉnh
            suggestion = checker.get_correction_suggestion()
            
            # In thông tin ra console
            rospy.loginfo_throttle(1, 
                f"Valid: {is_valid}, Angle: {angle:.1f if angle else 'N/A'}°, "
                f"Confidence: {confidence}, Action: {suggestion['action']}")
            
            # Hiển thị visualization (tùy chọn)
            vis_image = checker.visualize_angle_info()
            if vis_image is not None:
                cv2.imshow("Angle Checker", vis_image)
                cv2.waitKey(1)
            
            rate.sleep()
    
    except rospy.ROSInterruptException:
        rospy.loginfo("AngleChecker node đã bị ngắt.")
    except Exception as e:
        rospy.logerr(f"Lỗi trong AngleChecker: {e}")
    finally:
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()