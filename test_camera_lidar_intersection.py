#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
import time
from sensor_msgs.msg import Image
from opposite_detector import SimpleOppositeDetector

class CameraLidarIntersectionTest:
    def __init__(self):
        """Test camera-LiDAR intersection detection system."""
        rospy.init_node('camera_lidar_intersection_test', anonymous=True)
        
        # Camera parameters (same as main controller)
        self.WIDTH, self.HEIGHT = 300, 300
        self.ROI_Y = int(self.HEIGHT * 0.85)
        self.ROI_H = int(self.HEIGHT * 0.15)
        self.LINE_COLOR_LOWER = np.array([0, 0, 0])
        self.LINE_COLOR_UPPER = np.array([180, 255, 75])
        self.SCAN_PIXEL_THRESHOLD = 100
        
        # Camera intersection detection parameters
        self.CROSS_DETECTION_ROI_Y_PERCENT = 0.50
        self.CROSS_DETECTION_ROI_H_PERCENT = 0.20
        self.CROSS_MIN_ASPECT_RATIO = 2.0
        self.CROSS_MIN_WIDTH_RATIO = 0.4
        self.CROSS_MAX_HEIGHT_RATIO = 0.8
        
        # State tracking
        self.latest_image = None
        self.camera_intersection_detected = False
        self.camera_detection_time = 0
        self.waiting_for_lidar_confirmation = False
        self.lidar_confirmation_timeout = 3.0
        
        # LiDAR detector
        self.detector = SimpleOppositeDetector()
        self.detector.start_scanning()
        
        # ROS subscribers
        rospy.Subscriber('/csi_cam_0/image_raw', Image, self.camera_callback)
        
        rospy.loginfo("🔧 Camera-LiDAR Intersection Test initialized")
        rospy.loginfo("Camera will detect intersections first, then wait for LiDAR confirmation")
        
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
    
    def get_line_center(self, image, roi_y, roi_h):
        """Lấy center của line chính."""
        if image is None:
            return None
            
        roi = image[roi_y:roi_y + roi_h, :]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        color_mask = cv2.inRange(hsv, self.LINE_COLOR_LOWER, self.LINE_COLOR_UPPER)
        
        # Tìm contours
        _, contours, _ = cv2.findContours(color_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if not contours:
            return None
            
        largest_contour = max(contours, key=cv2.contourArea)
        
        if cv2.contourArea(largest_contour) < self.SCAN_PIXEL_THRESHOLD:
            return None
        
        M = cv2.moments(largest_contour)
        if M["m00"] > 0:
            return int(M["m10"] / M["m00"])
        
        return None
    
    def detect_camera_intersection(self):
        """
        Phát hiện giao lộ từ camera bằng cách tìm đường ngang vuông góc.
        """
        if self.latest_image is None:
            return False, "NO_IMAGE", None
        
        # Lấy ROI để tìm đường ngang
        cross_detection_roi_y = int(self.HEIGHT * self.CROSS_DETECTION_ROI_Y_PERCENT)
        cross_detection_roi_h = int(self.HEIGHT * self.CROSS_DETECTION_ROI_H_PERCENT)
        
        roi = self.latest_image[cross_detection_roi_y:cross_detection_roi_y + cross_detection_roi_h, :]
        
        # Tạo mask cho line
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        line_mask = cv2.inRange(hsv, self.LINE_COLOR_LOWER, self.LINE_COLOR_UPPER)
        
        # Morphological operations
        kernel = np.ones((3,3), np.uint8)
        line_mask = cv2.morphologyEx(line_mask, cv2.MORPH_CLOSE, kernel)
        line_mask = cv2.morphologyEx(line_mask, cv2.MORPH_OPEN, kernel)
        
        # Tìm contours
        _, contours, _ = cv2.findContours(line_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if not contours:
            return False, "NO_CONTOURS", None
        
        # Kiểm tra main line
        main_line_center = self.get_line_center(self.latest_image, self.ROI_Y, self.ROI_H)
        if main_line_center is None:
            return False, "NO_MAIN_LINE", None
        
        # Tìm cross candidates
        roi_height, roi_width = line_mask.shape
        cross_candidates = []
        
        for contour in contours:
            x, y, w, h = cv2.boundingRect(contour)
            aspect_ratio = w / h if h > 0 else 0
            width_ratio = w / roi_width
            height_ratio = h / roi_height
            
            if (aspect_ratio > self.CROSS_MIN_ASPECT_RATIO and 
                width_ratio > self.CROSS_MIN_WIDTH_RATIO and 
                height_ratio < self.CROSS_MAX_HEIGHT_RATIO):
                
                M = cv2.moments(contour)
                if M["m00"] > 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    
                    cross_candidates.append({
                        'contour': contour,
                        'center_x': cx,
                        'center_y': cy,
                        'width': w,
                        'height': h,
                        'aspect_ratio': aspect_ratio,
                        'area': cv2.contourArea(contour)
                    })
        
        if not cross_candidates:
            return False, "NO_CROSS_CANDIDATES", None
        
        # Chọn candidate tốt nhất
        best_candidate = None
        best_score = 0
        
        for candidate in cross_candidates:
            area_score = candidate['area'] / (roi_width * roi_height)
            center_score = 1.0 - abs(candidate['center_x'] - roi_width/2) / (roi_width/2)
            total_score = area_score * 0.6 + center_score * 0.4
            
            if total_score > best_score:
                best_score = total_score
                best_candidate = candidate
        
        if best_candidate is None:
            return False, "NO_GOOD_CANDIDATE", None
        
        # Confidence
        confidence_level = "LOW"
        if best_score > 0.7:
            confidence_level = "HIGH"
        elif best_score > 0.4:
            confidence_level = "MEDIUM"
        
        cross_line_center = best_candidate['center_x']
        return True, confidence_level, cross_line_center
    
    def check_camera_lidar_intersection(self):
        """Logic camera-first intersection detection."""
        current_time = rospy.get_time()
        
        # Camera detection
        if not self.camera_intersection_detected and not self.waiting_for_lidar_confirmation:
            camera_detected, camera_conf, cross_center = self.detect_camera_intersection()
            
            if camera_detected:
                print(f"\n📷 CAMERA DETECTION:")
                print(f"   • Confidence: {camera_conf}")
                print(f"   • Cross center: {cross_center}")
                print(f"   • Main line center: {self.get_line_center(self.latest_image, self.ROI_Y, self.ROI_H)}")
                
                if camera_conf in ["HIGH", "MEDIUM"]:
                    self.camera_intersection_detected = True
                    self.camera_detection_time = current_time
                    self.waiting_for_lidar_confirmation = True
                    print(f"📷 WAITING for LiDAR confirmation...")
                    return False
        
        # LiDAR confirmation
        if self.waiting_for_lidar_confirmation:
            # Timeout check
            if current_time - self.camera_detection_time > self.lidar_confirmation_timeout:
                print(f"⏰ TIMEOUT: LiDAR confirmation timeout, resetting")
                self.reset_intersection_detection()
                return False
            
            # LiDAR check
            if self.detector.process_detection():
                print(f"\n✅ INTERSECTION CONFIRMED!")
                print(f"📡 LiDAR confirmed camera detection")
                print(f"🎯 Ready to execute intersection handling")
                self.reset_intersection_detection()
                return True
            else:
                elapsed = current_time - self.camera_detection_time
                if elapsed % 1.0 < 0.1:  # Print every second
                    print(f"⏳ Waiting for LiDAR... ({elapsed:.1f}s/{self.lidar_confirmation_timeout}s)")
                return False
        
        return False
    
    def reset_intersection_detection(self):
        """Reset detection state."""
        self.camera_intersection_detected = False
        self.camera_detection_time = 0
        self.waiting_for_lidar_confirmation = False
    
    def visualize_detection(self):
        """Visualize detection results on image."""
        if self.latest_image is None:
            return None
        
        vis_image = self.latest_image.copy()
        
        # Draw ROIs
        cross_roi_y = int(self.HEIGHT * self.CROSS_DETECTION_ROI_Y_PERCENT)
        cross_roi_h = int(self.HEIGHT * self.CROSS_DETECTION_ROI_H_PERCENT)
        
        # Cross detection ROI (yellow)
        cv2.rectangle(vis_image, (0, cross_roi_y), (self.WIDTH-1, cross_roi_y + cross_roi_h), (0, 255, 255), 2)
        
        # Main line ROI (green)
        cv2.rectangle(vis_image, (0, self.ROI_Y), (self.WIDTH-1, self.ROI_Y + self.ROI_H), (0, 255, 0), 1)
        
        # Main line center
        main_center = self.get_line_center(self.latest_image, self.ROI_Y, self.ROI_H)
        if main_center is not None:
            cv2.line(vis_image, (main_center, self.ROI_Y), (main_center, self.ROI_Y + self.ROI_H), (0, 0, 255), 2)
        
        # Camera detection status
        status_text = "IDLE"
        if self.waiting_for_lidar_confirmation:
            status_text = "WAITING_LIDAR"
        elif self.camera_intersection_detected:
            status_text = "CAMERA_DETECTED"
        
        cv2.putText(vis_image, f"Status: {status_text}", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        return vis_image
    
    def run_test(self, show_visualization=False):
        """Run the intersection detection test."""
        rospy.loginfo("🚀 Starting camera-LiDAR intersection test")
        rospy.loginfo("Press Ctrl+C to stop")
        
        rate = rospy.Rate(10)  # 10 Hz
        
        try:
            while not rospy.is_shutdown():
                # Check intersection
                intersection_confirmed = self.check_camera_lidar_intersection()
                
                if intersection_confirmed:
                    print(f"\n🎉 INTERSECTION CONFIRMED! Both sensors agree.")
                    print(f"   This would trigger intersection handling in the main system")
                    print(f"   Waiting 5 seconds before continuing...\n")
                    time.sleep(5)
                
                # Visualization
                if show_visualization and self.latest_image is not None:
                    vis_image = self.visualize_detection()
                    if vis_image is not None:
                        cv2.imshow("Camera-LiDAR Intersection Detection", vis_image)
                        cv2.waitKey(1)
                
                rate.sleep()
                
        except rospy.ROSInterruptException:
            rospy.loginfo("🛑 Test stopped")
        finally:
            self.detector.stop_scanning()
            if show_visualization:
                cv2.destroyAllWindows()


def main():
    """Main function"""
    import sys
    
    show_viz = '--viz' in sys.argv or '--visual' in sys.argv
    
    try:
        tester = CameraLidarIntersectionTest()
        tester.run_test(show_visualization=show_viz)
        
    except rospy.ROSInterruptException:
        rospy.loginfo("Test interrupted")
    except Exception as e:
        rospy.logerr(f"Test error: {e}")


if __name__ == '__main__':
    main()