#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
import math
from sensor_msgs.msg import LaserScan, Image
from opposite_detector import SimpleOppositeDetector

class AngleCalculator:
    def __init__(self):
        """
        Khởi tạo calculator để tính góc giữa line từ camera và cluster từ LiDAR.
        """
        # Camera parameters
        self.WIDTH, self.HEIGHT = 300, 300
        self.ROI_Y = int(self.HEIGHT * 0.85)
        self.ROI_H = int(self.HEIGHT * 0.15)
        self.ROI_CENTER_WIDTH_PERCENT = 0.5
        self.LINE_COLOR_LOWER = np.array([0, 0, 0])
        self.LINE_COLOR_UPPER = np.array([180, 255, 75])
        self.SCAN_PIXEL_THRESHOLD = 100
        
        # LiDAR parameters
        self.detector = SimpleOppositeDetector()
        
        # Current data
        self.latest_image = None
        
        # ROS subscribers
        rospy.Subscriber('/scan', LaserScan, self.detector.callback)
        rospy.Subscriber('/csi_cam_0/image_raw', Image, self.camera_callback)
        
        rospy.loginfo("AngleCalculator initialized. Waiting for data...")
    
    def camera_callback(self, image_msg):
        """Callback xử lý ảnh từ camera."""
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
            rospy.logerr(f"Camera callback error: {e}")
    
    def get_line_center(self, image):
        """Lấy vị trí trung tâm của line từ camera."""
        if image is None:
            return None
            
        roi = image[self.ROI_Y:self.ROI_Y + self.ROI_H, :]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        
        # Tạo color mask
        color_mask = cv2.inRange(hsv, self.LINE_COLOR_LOWER, self.LINE_COLOR_UPPER)
        
        # Tạo focus mask (chỉ tập trung vào giữa)
        focus_mask = np.zeros_like(color_mask)
        roi_height, roi_width = focus_mask.shape
        center_width = int(roi_width * self.ROI_CENTER_WIDTH_PERCENT)
        start_x = (roi_width - center_width) // 2
        end_x = start_x + center_width
        cv2.rectangle(focus_mask, (start_x, 0), (end_x, roi_height), 255, -1)
        
        # Kết hợp hai mask
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
    
    def calculate_camera_angle(self):
        """Tính góc của line từ camera."""
        if self.latest_image is None:
            return None
            
        line_center = self.get_line_center(self.latest_image)
        if line_center is None:
            return None
        
        image_center = self.WIDTH / 2
        pixel_offset = line_center - image_center
        
        # Chuyển đổi pixel offset thành góc (FOV 60 degrees)
        camera_fov = 60
        angle = (pixel_offset / image_center) * (camera_fov / 2)
        
        return angle
    
    def find_line_clusters_in_lidar(self):
        """Tìm các clusters có thể là line trong dữ liệu LiDAR."""
        if self.detector.latest_scan is None:
            return []
        
        scan = self.detector.latest_scan
        ranges = np.array(scan.ranges)
        n = len(ranges)
        
        angle_increment_deg = math.degrees(scan.angle_increment)
        points_per_range = int(self.detector.angle_range / angle_increment_deg)
        
        line_clusters = []
        front_angle_range = 45  # ±45 degrees
        
        # Quét theo từng zone ở phía trước
        for start_idx in range(0, n, points_per_range // 2):
            end_idx = min(start_idx + points_per_range, n)
            if end_idx - start_idx < points_per_range // 2:
                continue
            
            center_idx = start_idx + (end_idx - start_idx) // 2
            center_angle = self.detector.index_to_angle(center_idx, scan)
            
            # Chỉ xét các zone ở phía trước
            if abs(center_angle) <= front_angle_range:
                cluster = self.detect_line_cluster_in_zone(ranges[start_idx:end_idx], center_angle)
                if cluster:
                    line_clusters.append(cluster)
        
        return line_clusters
    
    def detect_line_cluster_in_zone(self, zone_ranges, center_angle):
        """Phát hiện line cluster trong một zone."""
        if len(zone_ranges) == 0:
            return None
        
        # Lọc các điểm hợp lệ
        min_dist, max_dist = 0.15, 2.0
        valid_mask = ((zone_ranges >= min_dist) & 
                     (zone_ranges <= max_dist) & 
                     np.isfinite(zone_ranges))
        
        if np.sum(valid_mask) < 5:  # Ít nhất 5 điểm
            return None
        
        valid_ranges = zone_ranges[valid_mask]
        
        # Kiểm tra đặc điểm line: khoảng cách tương đối đồng đều
        distance_variance = np.var(valid_ranges)
        
        if distance_variance <= 0.4:  # Line có variance thấp
            return {
                'center_angle': center_angle,
                'distance': np.mean(valid_ranges),
                'point_count': len(valid_ranges),
                'variance': distance_variance
            }
        
        return None
    
    def calculate_lidar_angle(self):
        """Tính góc của line dựa trên các clusters từ LiDAR."""
        line_clusters = self.find_line_clusters_in_lidar()
        
        if len(line_clusters) < 1:
            return None
        
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
        if np.sum(weights) > 0:
            weights = weights / np.sum(weights)  # Normalize
            weighted_angle = np.average(angles, weights=weights)
        else:
            weighted_angle = np.mean(angles)
        
        return weighted_angle
    
    def get_angle_difference(self):
        """
        Trả về góc hợp bởi cluster LiDAR và line từ camera.
        
        Returns:
            float: Góc chênh lệch (degrees), None nếu không có dữ liệu
        """
        camera_angle = self.calculate_camera_angle()
        lidar_angle = self.calculate_lidar_angle()
        
        if camera_angle is None or lidar_angle is None:
            return None
        
        # Tính góc hợp bởi hai vector
        angle_difference = abs(camera_angle - lidar_angle)
        
        return angle_difference
    
    def get_angles(self):
        """
        Trả về cả hai góc và góc hợp.
        
        Returns:
            dict: {'camera_angle': float, 'lidar_angle': float, 'difference': float}
                  hoặc None nếu không có dữ liệu
        """
        camera_angle = self.calculate_camera_angle()
        lidar_angle = self.calculate_lidar_angle()
        
        if camera_angle is None or lidar_angle is None:
            return None
        
        difference = abs(camera_angle - lidar_angle)
        
        return {
            'camera_angle': camera_angle,
            'lidar_angle': lidar_angle,
            'difference': difference
        }


def main():
    """Test function"""
    rospy.init_node('angle_calculator_node', anonymous=True)
    
    try:
        calculator = AngleCalculator()
        rate = rospy.Rate(2)  # 2 Hz
        
        rospy.loginfo("Starting angle calculation...")
        
        while not rospy.is_shutdown():
            # Lấy góc hợp
            angle_diff = calculator.get_angle_difference()
            
            if angle_diff is not None:
                rospy.loginfo(f"Angle between camera and LiDAR: {angle_diff:.2f} degrees")
            else:
                rospy.logwarn("No valid data from camera or LiDAR")
            
            rate.sleep()
            
    except rospy.ROSInterruptException:
        rospy.loginfo("Node interrupted")
    except Exception as e:
        rospy.logerr(f"Error: {e}")


if __name__ == '__main__':
    main()