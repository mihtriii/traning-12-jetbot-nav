#!/usr/bin/env python3

import rospy
import numpy as np
import math
import cv2
from sensor_msgs.msg import LaserScan, Image
from opposite_detector import SimpleOppositeDetector

class LineAngleAnalyzer:
    def __init__(self):
        """
        Phân tích góc hợp bởi line từ camera ROS và cluster từ LiDAR.
        """
        rospy.loginfo("Khởi tạo LineAngleAnalyzer...")
        
        # Parameters từ ros_lidar_follower.py
        self.WIDTH, self.HEIGHT = 300, 300
        self.ROI_Y = int(self.HEIGHT * 0.85)
        self.ROI_H = int(self.HEIGHT * 0.15)
        self.CAMERA_FOV = 60  # Field of view của camera (degrees)
        
        # Line detection parameters
        self.LINE_COLOR_LOWER = np.array([0, 0, 0])
        self.LINE_COLOR_UPPER = np.array([180, 255, 75])
        self.SCAN_PIXEL_THRESHOLD = 100
        self.ROI_CENTER_WIDTH_PERCENT = 0.5
        
        # LiDAR parameters (từ opposite_detector)
        self.min_distance = 0.15
        self.max_distance = 1.5  # Tăng lên để detect line xa hơn
        self.angle_range = 10.0
        self.distance_threshold = 0.1
        self.object_min_points = 5
        
        # Line angle detection parameters
        self.LINE_FRONT_ANGLE_RANGE = 45  # ±45 degrees từ phía trước
        self.LINE_DISTANCE_VARIANCE_THRESHOLD = 0.3  # Độ biến thiên cho line
        self.MIN_LINE_POINTS = 6
        
        # Data storage
        self.latest_scan = None
        self.latest_image = None
        
        # Initialize detector
        self.detector = SimpleOppositeDetector()
        
        # ROS subscribers
        rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        rospy.Subscriber('/csi_cam_0/image_raw', Image, self.camera_callback)
        
        rospy.loginfo("LineAngleAnalyzer đã sẵn sàng!")
    
    def scan_callback(self, scan):
        """Callback để nhận dữ liệu LiDAR."""
        self.latest_scan = scan
    
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
        Tìm trọng tâm của line từ camera (giống logic trong ros_lidar_follower.py).
        """
        if self.latest_image is None:
            return None
        
        roi = self.latest_image[self.ROI_Y : self.ROI_Y + self.ROI_H, :]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        
        # Tạo mặt nạ màu sắc
        color_mask = cv2.inRange(hsv, self.LINE_COLOR_LOWER, self.LINE_COLOR_UPPER)
        
        # Tạo mặt nạ tập trung (focus mask)
        focus_mask = np.zeros_like(color_mask)
        roi_height, roi_width = focus_mask.shape
        
        center_width = int(roi_width * self.ROI_CENTER_WIDTH_PERCENT)
        start_x = (roi_width - center_width) // 2
        end_x = start_x + center_width
        
        cv2.rectangle(focus_mask, (start_x, 0), (end_x, roi_height), 255, -1)
        
        # Kết hợp hai mặt nạ
        final_mask = cv2.bitwise_and(color_mask, focus_mask)
        
        # Tìm contours
        _, contours, _ = cv2.findContours(final_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if not contours:
            return None
            
        largest_contour = max(contours, key=cv2.contourArea)
        
        if cv2.contourArea(largest_contour) < self.SCAN_PIXEL_THRESHOLD:
            return None
        
        # Tính trọng tâm
        M = cv2.moments(largest_contour)
        if M["m00"] > 0:
            return int(M["m10"] / M["m00"])
        
        return None
    
    def calculate_line_angle_from_camera(self):
        """
        Tính góc của line từ camera dựa trên vị trí trọng tâm.
        """
        line_center = self.get_line_center_from_camera()
        if line_center is None:
            return None, "NO_LINE_DETECTED"
        
        image_center = self.WIDTH / 2
        pixel_offset = line_center - image_center
        
        # Chuyển đổi pixel offset thành góc
        angle = (pixel_offset / image_center) * (self.CAMERA_FOV / 2)
        
        # Đánh giá confidence
        if abs(angle) < 3:
            confidence = "HIGH"
        elif abs(angle) < 10:
            confidence = "MEDIUM"
        else:
            confidence = "LOW"
        
        return angle, confidence
    
    def index_to_angle(self, index, scan):
        """Chuyển đổi index thành góc (degrees)."""
        angle_rad = scan.angle_min + (index * scan.angle_increment)
        return math.degrees(angle_rad)
    
    def find_line_clusters_in_lidar(self):
        """
        Tìm các clusters có thể là line trong dữ liệu LiDAR.
        """
        if self.latest_scan is None:
            return []
        
        scan = self.latest_scan
        ranges = np.array(scan.ranges)
        n = len(ranges)
        
        angle_increment_deg = math.degrees(scan.angle_increment)
        points_per_range = int(self.angle_range / angle_increment_deg)
        
        line_clusters = []
        
        # Quét theo từng zone
        for start_idx in range(0, n, points_per_range // 2):
            end_idx = min(start_idx + points_per_range, n)
            if end_idx - start_idx < points_per_range // 2:
                continue
            
            zone_ranges = ranges[start_idx:end_idx]
            center_idx = start_idx + (end_idx - start_idx) // 2
            center_angle = self.index_to_angle(center_idx, scan)
            
            # Chỉ xét các zone ở phía trước
            if abs(center_angle) <= self.LINE_FRONT_ANGLE_RANGE:
                cluster = self.detect_line_cluster_in_zone(zone_ranges, center_angle, start_idx)
                if cluster:
                    line_clusters.append(cluster)
        
        return line_clusters
    
    def detect_line_cluster_in_zone(self, zone_ranges, center_angle, start_idx):
        """
        Phát hiện line cluster trong một zone cụ thể.
        """
        if len(zone_ranges) == 0:
            return None
        
        # Lọc các điểm hợp lệ
        valid_mask = ((zone_ranges >= self.min_distance) & 
                     (zone_ranges <= self.max_distance) & 
                     np.isfinite(zone_ranges))
        
        if np.sum(valid_mask) < self.object_min_points:
            return None
        
        valid_ranges = zone_ranges[valid_mask]
        valid_indices = np.where(valid_mask)[0]
        
        # Tìm clusters
        clusters = []
        current_cluster = [0]
        
        for i in range(1, len(valid_ranges)):
            if (valid_indices[i] - valid_indices[current_cluster[-1]] <= 2 and 
                abs(valid_ranges[i] - valid_ranges[current_cluster[-1]]) <= self.distance_threshold):
                current_cluster.append(i)
            else:
                if len(current_cluster) >= self.object_min_points:
                    clusters.append(current_cluster)
                current_cluster = [i]
        
        if len(current_cluster) >= self.object_min_points:
            clusters.append(current_cluster)
        
        if not clusters:
            return None
        
        # Chọn cluster lớn nhất
        largest_cluster = max(clusters, key=len)
        cluster_distances = [valid_ranges[i] for i in largest_cluster]
        
        # Kiểm tra đặc điểm line: khoảng cách tương đối đồng đều
        distance_variance = np.var(cluster_distances)
        
        if distance_variance <= self.LINE_DISTANCE_VARIANCE_THRESHOLD:
            return {
                'center_angle': center_angle,
                'distance': np.mean(cluster_distances),
                'point_count': len(largest_cluster),
                'variance': distance_variance,
                'start_idx': start_idx,
                'distances': cluster_distances
            }
        
        return None
    
    def calculate_line_angle_from_lidar(self):
        """
        Tính góc của line dựa trên các clusters từ LiDAR.
        """
        line_clusters = self.find_line_clusters_in_lidar()
        
        if len(line_clusters) < 1:
            return None, "NO_CLUSTERS_FOUND"
        
        # Weighted average dựa trên số điểm và khoảng cách
        angles = [cluster['center_angle'] for cluster in line_clusters]
        distances = [cluster['distance'] for cluster in line_clusters]
        point_counts = [cluster['point_count'] for cluster in line_clusters]
        
        # Tính trọng số (gần hơn và nhiều điểm hơn = trọng số cao hơn)
        weights = []
        for i in range(len(line_clusters)):
            weight = point_counts[i] / (distances[i] + 0.1)
            weights.append(weight)
        
        weights = np.array(weights)
        weights = weights / np.sum(weights)  # Normalize
        
        weighted_angle = np.average(angles, weights=weights)
        
        # Lọc nhiễu
        filtered_angles = []
        for i, angle in enumerate(angles):
            if abs(angle - weighted_angle) < 15:  # ±15 degrees tolerance
                filtered_angles.extend([angle] * point_counts[i])
        
        if len(filtered_angles) < self.MIN_LINE_POINTS:
            return weighted_angle, "LOW_CONFIDENCE"
        
        final_angle = np.mean(filtered_angles)
        
        # Đánh giá confidence
        if len(line_clusters) >= 3 and len(filtered_angles) >= 10:
            confidence = "HIGH"
        elif len(line_clusters) >= 2:
            confidence = "MEDIUM"
        else:
            confidence = "LOW"
        
        return final_angle, confidence
    
    def analyze_line_angles_combined(self):
        """
        Phân tích và so sánh góc line từ camera và LiDAR.
        """
        camera_angle, camera_conf = self.calculate_line_angle_from_camera()
        lidar_angle, lidar_conf = self.calculate_line_angle_from_lidar()
        
        analysis_result = {
            'timestamp': rospy.get_time(),
            'camera': {
                'angle': camera_angle,
                'confidence': camera_conf
            },
            'lidar': {
                'angle': lidar_angle,
                'confidence': lidar_conf
            },
            'angle_difference': None,
            'agreement_level': None,
            'combined_angle': None,
            'recommendation': None
        }
        
        # Tính toán sự khác biệt góc
        if camera_angle is not None and lidar_angle is not None:
            angle_diff = abs(camera_angle - lidar_angle)
            analysis_result['angle_difference'] = angle_diff
            
            # Đánh giá mức độ đồng thuận
            if angle_diff < 3:
                agreement = "EXCELLENT"
            elif angle_diff < 8:
                agreement = "GOOD"
            elif angle_diff < 15:
                agreement = "FAIR"
            else:
                agreement = "POOR"
            
            analysis_result['agreement_level'] = agreement
            
            # Góc kết hợp
            if camera_conf == "HIGH" and lidar_conf == "HIGH":
                combined_angle = (camera_angle + lidar_angle) / 2
                recommendation = "USE_COMBINED"
            elif camera_conf == "HIGH":
                combined_angle = camera_angle
                recommendation = "PREFER_CAMERA"
            elif lidar_conf == "HIGH":
                combined_angle = lidar_angle
                recommendation = "PREFER_LIDAR"
            else:
                combined_angle = (camera_angle + lidar_angle) / 2
                recommendation = "USE_COMBINED_CAUTION"
            
            analysis_result['combined_angle'] = combined_angle
            analysis_result['recommendation'] = recommendation
            
        elif camera_angle is not None:
            analysis_result['combined_angle'] = camera_angle
            analysis_result['recommendation'] = "CAMERA_ONLY"
            analysis_result['agreement_level'] = "N/A"
            
        elif lidar_angle is not None:
            analysis_result['combined_angle'] = lidar_angle
            analysis_result['recommendation'] = "LIDAR_ONLY"
            analysis_result['agreement_level'] = "N/A"
            
        else:
            analysis_result['recommendation'] = "NO_DATA"
            analysis_result['agreement_level'] = "N/A"
        
        return analysis_result
    
    def print_detailed_analysis(self):
        """
        In ra phân tích chi tiết về góc line.
        """
        analysis = self.analyze_line_angles_combined()
        
        print("\n" + "="*80)
        print("🔍 LINE ANGLE ANALYSIS - DETAILED REPORT")
        print(f"Timestamp: {analysis['timestamp']:.2f}s")
        print("="*80)
        
        # Camera Analysis
        print(f"📷 CAMERA ANALYSIS:")
        if analysis['camera']['angle'] is not None:
            print(f"   • Angle: {analysis['camera']['angle']:+6.2f}°")
            print(f"   • Confidence: {analysis['camera']['confidence']}")
            
            # Giải thích ý nghĩa góc
            if analysis['camera']['angle'] > 5:
                direction = "RIGHT (line is to the right of robot center)"
            elif analysis['camera']['angle'] < -5:
                direction = "LEFT (line is to the left of robot center)"
            else:
                direction = "STRAIGHT (line is centered)"
            print(f"   • Direction: {direction}")
        else:
            print(f"   • Status: {analysis['camera']['confidence']}")
        
        # LiDAR Analysis
        print(f"\n📡 LIDAR ANALYSIS:")
        if analysis['lidar']['angle'] is not None:
            print(f"   • Angle: {analysis['lidar']['angle']:+6.2f}°")
            print(f"   • Confidence: {analysis['lidar']['confidence']}")
            
            # Thông tin clusters
            clusters = self.find_line_clusters_in_lidar()
            print(f"   • Line clusters found: {len(clusters)}")
            for i, cluster in enumerate(clusters):
                print(f"     Cluster {i+1}: {cluster['point_count']} points at {cluster['center_angle']:+6.1f}°, "
                      f"avg dist: {cluster['distance']:.3f}m, variance: {cluster['variance']:.4f}")
        else:
            print(f"   • Status: {analysis['lidar']['confidence']}")
        
        # Comparison Analysis
        print(f"\n🔄 COMPARISON ANALYSIS:")
        if analysis['angle_difference'] is not None:
            print(f"   • Angle difference: {analysis['angle_difference']:.2f}°")
            print(f"   • Agreement level: {analysis['agreement_level']}")
            
            # Detailed comparison
            camera_angle = analysis['camera']['angle']
            lidar_angle = analysis['lidar']['angle']
            
            if camera_angle is not None and lidar_angle is not None:
                if abs(camera_angle - lidar_angle) > 10:
                    print(f"   ⚠️  WARNING: Large discrepancy between sensors!")
                    if abs(camera_angle) > abs(lidar_angle):
                        print(f"     Camera suggests more aggressive turn than LiDAR")
                    else:
                        print(f"     LiDAR suggests more aggressive turn than Camera")
        else:
            print(f"   • Agreement level: {analysis['agreement_level']}")
        
        # Final Recommendation
        print(f"\n💡 RECOMMENDATION:")
        print(f"   • Strategy: {analysis['recommendation']}")
        if analysis['combined_angle'] is not None:
            print(f"   • Combined angle: {analysis['combined_angle']:+6.2f}°")
            
            # Action recommendation
            abs_angle = abs(analysis['combined_angle'])
            if abs_angle < 2:
                action = "CONTINUE_STRAIGHT"
            elif abs_angle < 8:
                action = "SLIGHT_CORRECTION"
            elif abs_angle < 15:
                action = "MODERATE_CORRECTION" 
            else:
                action = "STRONG_CORRECTION"
            
            direction = "LEFT" if analysis['combined_angle'] < 0 else "RIGHT"
            print(f"   • Suggested action: {action} {direction}")
        
        print("="*80)
        print("🔍 END OF LINE ANGLE ANALYSIS")
        print("="*80 + "\n")
        
        return analysis

def main():
    """Test function để chạy LineAngleAnalyzer."""
    rospy.init_node('line_angle_analyzer_node', anonymous=True)
    
    try:
        analyzer = LineAngleAnalyzer()
        rate = rospy.Rate(2)  # 2 Hz để không spam
        
        rospy.loginfo("Bắt đầu phân tích góc line. Nhấn Ctrl+C để dừng.")
        
        while not rospy.is_shutdown():
            if analyzer.latest_scan is not None and analyzer.latest_image is not None:
                # In phân tích chi tiết
                analyzer.print_detailed_analysis()
            else:
                rospy.logwarn_throttle(5, "Đang chờ dữ liệu từ LiDAR và Camera...")
            
            rate.sleep()
    
    except rospy.ROSInterruptException:
        rospy.loginfo("LineAngleAnalyzer node đã bị ngắt.")
    except Exception as e:
        rospy.logerr(f"Lỗi trong LineAngleAnalyzer: {e}")

if __name__ == '__main__':
    main()