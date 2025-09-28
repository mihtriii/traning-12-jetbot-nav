#!/usr/bin/env python3
"""
YOLO Sign Follower - JetBot điều hướng dựa trên biển báo YOLO
Refactored từ ros_lidar_follower (1).py để không sử dụng map
Tác giả: AI Assistant
Ngày: 28/09/2025

Yêu cầu:
- N,E,W,S: Hướng tuyệt đối - BẮT BUỘC theo
- NN,NE,NW,NS: Biển cấm - KHÔNG ĐƯỢC theo
- L: Đích đến
- Robot bắt đầu hướng Đông
- Biển báo ở phía Tây Bắc
"""

import rospy
import time
import cv2
import numpy as np
import math
import json
import onnxruntime as ort
import paho.mqtt.client as mqtt
from enum import Enum
# from cv_bridge import CvBridge  # REMOVED - causes PyInit_cv_bridge_boost error
from jetbot import Robot
from pyzbar.pyzbar import decode
from sensor_msgs.msg import LaserScan, Image
from opposite_detector import SimpleOppositeDetector


class RobotState(Enum):
    WAITING_FOR_RED_FLAG = 0      # Chờ cờ đỏ để bắt đầu
    DRIVING_STRAIGHT = 1          # Đi thẳng theo line
    LINE_VALIDATION = 1.5         # Validate vị trí line
    APPROACHING_INTERSECTION = 2   # Tiến gần giao lộ
    SCANNING_SIGNS = 3            # Quét biển báo YOLO
    PROCESSING_DECISION = 4        # Xử lý quyết định từ biển báo
    TURNING = 5                   # Thực hiện rẽ
    GOAL_REACHED = 6              # Đã đến đích (phát hiện "L")
    DEAD_END = 7                  # Không tìm thấy đường đi


class AbsoluteDirection(Enum):
    NORTH, EAST, SOUTH, WEST = 0, 1, 2, 3


class YOLOSignFollower:
    def __init__(self):
        rospy.loginfo("🤖 Khởi tạo YOLO Sign Follower...")
        
        # Khởi tạo các thành phần
        self.setup_parameters()
        self.initialize_hardware()
        self.initialize_yolo()
        self.initialize_mqtt()
        self.initialize_video_writer()
        
        # Trạng thái robot
        self.current_state = None
        self.current_direction = AbsoluteDirection.EAST  # Bắt đầu hướng Đông
        self.goal_reached = False
        
        # Dữ liệu cảm biến
        self.latest_scan = None
        self.latest_image = None
        self.detector = SimpleOppositeDetector()
        
        # Biến điều khiển flag
        self.initial_red_flag_detected = False
        self.robot_started = False
        
        # Biến điều khiển intersection và YOLO
        self.camera_intersection_detected = False
        self.camera_detection_time = 0
        self.waiting_for_lidar_confirmation = False
        self.detected_signs = []
        self.current_decision = None
        
        # Angle analyzer cho line/lidar comparison
        self.angle_analysis_enabled = True
        self.last_angle_analysis_time = 0
        self.angle_analysis_interval = 2.0
        
        # Line validation
        self.line_validation_attempts = 0
        
        # Khởi tạo subscribers
        rospy.Subscriber('/scan', LaserScan, self.detector.callback)
        rospy.Subscriber('/csi_cam_0/image_raw', Image, self.camera_callback)
        
        self.state_change_time = rospy.get_time()
        self._set_state(RobotState.WAITING_FOR_RED_FLAG, initial=True)
        
        rospy.loginfo("✅ YOLO Sign Follower khởi tạo hoàn tất!")

    def setup_parameters(self):
        """Thiết lập các tham số cấu hình."""
        
        # === CAMERA & IMAGE ===
        self.WIDTH, self.HEIGHT = 224, 224
        self.ROI_Y = int(self.HEIGHT * 0.85)
        self.ROI_H = int(self.HEIGHT * 0.2)
        self.ROI_CENTER_WIDTH_PERCENT = 0.5
        self.LOOKAHEAD_ROI_Y = int(self.HEIGHT * 0.75)
        self.LOOKAHEAD_ROI_H = int(self.HEIGHT * 0.15)
        
        # === ROBOT CONTROL ===
        self.BASE_SPEED = 0.2
        self.TURN_SPEED = 0.2
        self.TURN_DURATION_90_DEG = 0.8
        self.CORRECTION_GAIN = 0.5
        self.SAFE_ZONE_PERCENT = 0.3
        self.MAX_CORRECTION_ADJ = 0.12
        
        # === LINE DETECTION ===
        self.LINE_COLOR_LOWER = np.array([0, 0, 0])
        self.LINE_COLOR_UPPER = np.array([180, 255, 120])
        self.SCAN_PIXEL_THRESHOLD = 100
        
        # === INTERSECTION DETECTION ===
        self.INTERSECTION_CLEARANCE_DURATION = 1.5
        self.INTERSECTION_APPROACH_DURATION = 0.5
        self.LINE_REACQUIRE_TIMEOUT = 3.0
        self.lidar_confirmation_timeout = 4.0
        
        # === YOLO CONFIGURATION ===
        self.YOLO_MODEL_PATH = "models/best.onnx"
        self.YOLO_CONF_THRESHOLD = 0.6
        self.YOLO_INPUT_SIZE = (640, 640)
        
        # === BIỂN BÁO SIGNS - QUAN TRỌNG ===
        # Biển báo hướng tuyệt đối - BẮT BUỘC theo
        self.DIRECTION_SIGNS = {'N', 'E', 'W', 'S'}
        
        # Biển báo cấm - KHÔNG ĐƯỢC theo
        self.PROHIBITION_SIGNS = {'NN', 'NE', 'NW', 'NS'}
        
        # Biển báo đích đến
        self.GOAL_SIGN = 'L'
        
        # Biển báo dữ liệu khác
        self.DATA_ITEMS = {'qr_code', 'math_problem'}
        
        self.YOLO_CLASS_NAMES = ['N', 'E', 'W', 'S', 'NN', 'NE', 'NW', 'NS', 'L', 'math', 'qr_code']
        
        # === DIRECTION MAPPING ===
        self.DIRECTIONS = [AbsoluteDirection.NORTH, AbsoluteDirection.EAST, 
                          AbsoluteDirection.SOUTH, AbsoluteDirection.WEST]
        
        self.LABEL_TO_DIRECTION_ENUM = {
            'N': AbsoluteDirection.NORTH, 
            'E': AbsoluteDirection.EAST, 
            'S': AbsoluteDirection.SOUTH, 
            'W': AbsoluteDirection.WEST
        }
        
        # === SIGN SCANNING ANGLES ===
        # Góc quay để nhìn về phía Tây Bắc (Northwest) từ mỗi hướng
        self.SIGN_SCAN_ANGLES = {
            AbsoluteDirection.NORTH: -45,   # Từ Bắc quay trái 45° để nhìn Tây Bắc
            AbsoluteDirection.EAST: -135,   # Từ Đông quay trái 135° để nhìn Tây Bắc
            AbsoluteDirection.SOUTH: 135,   # Từ Nam quay phải 135° để nhìn Tây Bắc
            AbsoluteDirection.WEST: 45      # Từ Tây quay phải 45° để nhìn Tây Bắc
        }
        
        # === LINE VALIDATION ===
        self.LINE_VALIDATION_TIMEOUT = 2.0
        self.LINE_CENTER_TOLERANCE = 0.2
        self.LINE_VALIDATION_ATTEMPTS = 8
        
        # === CAMERA-LIDAR INTERSECTION ===
        self.CAMERA_LIDAR_INTERSECTION_MODE = True
        self.CROSS_DETECTION_ROI_Y_PERCENT = 0.45
        self.CROSS_DETECTION_ROI_H_PERCENT = 0.30
        self.CROSS_MIN_ASPECT_RATIO = 1.2
        self.CROSS_MIN_WIDTH_RATIO = 0.25
        self.CROSS_MAX_HEIGHT_RATIO = 0.8
        
        # === RED FLAG DETECTION ===
        self.FLAG_RED_LOWER1 = np.array([0, 50, 50])
        self.FLAG_RED_UPPER1 = np.array([10, 255, 255])
        self.FLAG_RED_LOWER2 = np.array([170, 50, 50])
        self.FLAG_RED_UPPER2 = np.array([180, 255, 255])
        self.FLAG_COVERAGE_THRESHOLD = 0.15
        
        # === MQTT ===
        self.MQTT_BROKER = "localhost"
        self.MQTT_PORT = 1883
        self.MQTT_DATA_TOPIC = "jetbot/yolo_sign_data"
        
        # === VIDEO RECORDING ===
        self.VIDEO_OUTPUT_FILENAME = 'yolo_sign_follower_run.avi'
        self.VIDEO_FPS = 20
        self.VIDEO_FOURCC = cv2.VideoWriter_fourcc(*'MJPG')

    def initialize_hardware(self):
        """Khởi tạo robot hardware."""
        try:
            self.robot = Robot()
            rospy.loginfo("✅ Robot hardware initialized")
        except Exception as e:
            rospy.logerr(f"❌ Lỗi khởi tạo robot: {e}")

    def initialize_yolo(self):
        """Khởi tạo YOLO model."""
        try:
            self.yolo_session = ort.InferenceSession(self.YOLO_MODEL_PATH)
            rospy.loginfo(f"✅ YOLO model loaded: {self.YOLO_MODEL_PATH}")
        except Exception as e:
            rospy.logerr(f"❌ Lỗi tải YOLO model: {e}")
            self.yolo_session = None

    def initialize_mqtt(self):
        """Khởi tạo MQTT client."""
        self.mqtt_client = mqtt.Client()
        
        def on_connect(client, userdata, flags, rc):
            rospy.loginfo(f"MQTT connected với code {rc}")
            
        self.mqtt_client.on_connect = on_connect
        
        try:
            self.mqtt_client.connect(self.MQTT_BROKER, self.MQTT_PORT, 60)
            self.mqtt_client.loop_start()
        except Exception as e:
            rospy.logwarn(f"MQTT connection failed: {e}")

    def initialize_video_writer(self):
        """Khởi tạo video writer."""
        try:
            self.video_writer = cv2.VideoWriter(
                self.VIDEO_OUTPUT_FILENAME, 
                self.VIDEO_FOURCC, 
                self.VIDEO_FPS, 
                (self.WIDTH, self.HEIGHT)
            )
            rospy.loginfo(f"✅ Video writer initialized: {self.VIDEO_OUTPUT_FILENAME}")
        except Exception as e:
            rospy.logwarn(f"Video writer initialization failed: {e}")
            self.video_writer = None

    def _set_state(self, new_state, initial=False):
        """Thay đổi trạng thái robot."""
        if self.current_state != new_state:
            if not initial:
                elapsed = rospy.get_time() - self.state_change_time
                rospy.loginfo(f"🔄 State: {self.current_state.name} -> {new_state.name} (elapsed: {elapsed:.2f}s)")
            else:
                rospy.loginfo(f"🎯 Initial State: {new_state.name}")
                
            self.current_state = new_state
            self.state_change_time = rospy.get_time()

    def camera_callback(self, image_msg):
        """Callback xử lý dữ liệu camera - FIXED VERSION (no cv_bridge needed)."""
        try:
            # Trực tiếp convert từ ROS Image sang OpenCV bằng numpy
            if image_msg.encoding == "bgr8":
                np_arr = np.frombuffer(image_msg.data, np.uint8)
                cv_image = np_arr.reshape((image_msg.height, image_msg.width, 3))
            elif image_msg.encoding == "rgb8":  
                np_arr = np.frombuffer(image_msg.data, np.uint8)
                cv_image = np_arr.reshape((image_msg.height, image_msg.width, 3))
                cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)
            elif image_msg.encoding == "mono8":
                np_arr = np.frombuffer(image_msg.data, np.uint8) 
                cv_image = np_arr.reshape((image_msg.height, image_msg.width))
                cv_image = cv2.cvtColor(cv_image, cv2.COLOR_GRAY2BGR)
            else:
                rospy.logwarn(f"⚠️ Unsupported encoding: {image_msg.encoding}, trying default...")
                np_arr = np.frombuffer(image_msg.data, np.uint8)
                cv_image = np_arr.reshape((image_msg.height, image_msg.width, 3))
            
            # Resize về kích thước chuẩn
            self.latest_image = cv2.resize(cv_image, (self.WIDTH, self.HEIGHT))
            
            # Log lần đầu nhận được frame
            if not hasattr(self, '_camera_ok_logged'):
                rospy.loginfo(f"✅ Camera OK! {image_msg.width}x{image_msg.height}, {image_msg.encoding}")
                self._camera_ok_logged = True
                
        except Exception as e:
            if not hasattr(self, '_camera_error_count'):
                self._camera_error_count = 0
            self._camera_error_count += 1
            
            # Chỉ log error đầu tiên để tránh spam
            if self._camera_error_count == 1:
                rospy.logerr(f"❌ Camera callback error: {e}")
                rospy.logerr("Trying to continue without camera...")

    # ================ LINE FOLLOWING METHODS (Giữ nguyên) ================
    
    def _get_line_center(self, image, roi_y, roi_h):
        """Tìm tâm của line trong ROI - giữ nguyên từ code gốc."""
        if image is None:
            return None
            
        roi = image[roi_y : roi_y + roi_h, :]
        
        # HSV detection
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        color_mask = cv2.inRange(hsv, self.LINE_COLOR_LOWER, self.LINE_COLOR_UPPER)
        
        # Grayscale threshold detection
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        _, thresh_mask = cv2.threshold(gray, 60, 255, cv2.THRESH_BINARY_INV)
        
        # Combine both methods
        combined_mask = cv2.bitwise_or(color_mask, thresh_mask)
        
        # Focus mask
        focus_mask = np.zeros_like(combined_mask)
        roi_height, roi_width = focus_mask.shape
        
        center_width = int(roi_width * self.ROI_CENTER_WIDTH_PERCENT)
        start_x = (roi_width - center_width) // 2
        end_x = start_x + center_width
        
        cv2.rectangle(focus_mask, (start_x, 0), (end_x, roi_height), 255, -1)
        final_mask = cv2.bitwise_and(combined_mask, focus_mask)

        _, contours, _ = cv2.findContours(final_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if not contours:
            return None
            
        c = max(contours, key=cv2.contourArea)
        
        if cv2.contourArea(c) < self.SCAN_PIXEL_THRESHOLD:
            return None

        M = cv2.moments(c)
        if M["m00"] > 0:
            return int(M["m10"] / M["m00"])
            
        return None

    def _is_line_in_valid_range(self, image):
        """Kiểm tra line có trong phạm vi hợp lệ - giữ nguyên từ code gốc."""
        if image is None:
            return False
            
        line_center = self._get_line_center(image, self.ROI_Y, self.ROI_H)
        if line_center is None:
            return False
            
        image_center = self.WIDTH / 2
        max_deviation = self.WIDTH * self.LINE_CENTER_TOLERANCE
        deviation = abs(line_center - image_center)
        
        is_valid = deviation <= max_deviation
        
        rospy.loginfo(f"Line validation: center={line_center}, deviation={deviation:.1f}, max_allowed={max_deviation:.1f}, valid={is_valid}")
        return is_valid

    def correct_course(self, line_center_x):
        """Điều chỉnh hướng đi theo line - giữ nguyên từ code gốc."""
        error = line_center_x - (self.WIDTH / 2)
        
        # Đi thẳng nếu sai số nhỏ
        if abs(error) < (self.WIDTH / 2) * self.SAFE_ZONE_PERCENT:
            self.robot.set_motors(self.BASE_SPEED, self.BASE_SPEED)
            return

        # Tính toán lực điều chỉnh
        adj = (error / (self.WIDTH / 2)) * self.CORRECTION_GAIN
        adj = np.clip(adj, -self.MAX_CORRECTION_ADJ, self.MAX_CORRECTION_ADJ)
        
        # Áp dụng điều chỉnh
        left_motor = self.BASE_SPEED + adj
        right_motor = self.BASE_SPEED - adj
        self.robot.set_motors(left_motor, right_motor)

    # ================ INTERSECTION DETECTION (Giữ nguyên) ================
    
    def check_camera_lidar_intersection(self):
        """Kiểm tra giao lộ bằng camera + LiDAR - giữ nguyên từ code gốc."""
        current_time = rospy.get_time()
        
        # Bước 1: Camera detection
        if not self.camera_intersection_detected and not self.waiting_for_lidar_confirmation:
            camera_detected, camera_conf, cross_center = self.detect_camera_intersection()
            
            if camera_detected:
                rospy.loginfo(f"📷 CAMERA: Intersection detected! Confidence: {camera_conf}")
                if camera_conf in ["HIGH", "MEDIUM"]:
                    self.camera_intersection_detected = True
                    self.camera_detection_time = current_time
                    
                    rospy.loginfo("🚀 Moving forward to position LiDAR for better detection...")
                    self.move_forward_briefly()
                    
                    self.waiting_for_lidar_confirmation = True
                    rospy.loginfo("📷 CAMERA: Waiting for LiDAR confirmation...")
                    return False
        
        # Bước 2: Chờ LiDAR confirmation
        if self.waiting_for_lidar_confirmation:
            if current_time - self.camera_detection_time > self.lidar_confirmation_timeout:
                rospy.logwarn("⏰ TIMEOUT: LiDAR didn't confirm intersection, resetting")
                self.reset_intersection_detection()
                return False
            
            if self.detector.process_detection():
                rospy.loginfo("✅ INTERSECTION CONFIRMED: Both camera and LiDAR detected intersection!")
                self.reset_intersection_detection()
                return True
            else:
                elapsed = current_time - self.camera_detection_time
                rospy.loginfo(f"⏳ Waiting for LiDAR confirmation... ({elapsed:.1f}s/{self.lidar_confirmation_timeout}s)")
                return False
        
        return False

    def detect_camera_intersection(self):
        """Phát hiện intersection bằng camera - giữ nguyên từ code gốc với một số điều chỉnh."""
        if self.latest_image is None:
            return False, "LOW", None
        
        # Lấy ROI để tìm đường ngang
        cross_detection_roi_y = int(self.HEIGHT * self.CROSS_DETECTION_ROI_Y_PERCENT)
        cross_detection_roi_h = int(self.HEIGHT * self.CROSS_DETECTION_ROI_H_PERCENT)
        
        roi = self.latest_image[cross_detection_roi_y:cross_detection_roi_y + cross_detection_roi_h, :]
        
        # Multi-method detection
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        hsv_mask = cv2.inRange(hsv, self.LINE_COLOR_LOWER, self.LINE_COLOR_UPPER)
        
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        _, thresh_mask = cv2.threshold(gray, 60, 255, cv2.THRESH_BINARY_INV)
        
        adaptive_mask = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_MEAN_C, 
                                            cv2.THRESH_BINARY_INV, 11, 2)
        
        combined_mask = cv2.bitwise_or(hsv_mask, thresh_mask)
        combined_mask = cv2.bitwise_or(combined_mask, adaptive_mask)
        
        # Morphological operations
        kernels = [np.ones((2,2), np.uint8), np.ones((3,3), np.uint8), np.ones((4,4), np.uint8)]
        
        processed_masks = []
        for kernel in kernels:
            cleaned = cv2.morphologyEx(combined_mask, cv2.MORPH_CLOSE, kernel)
            cleaned = cv2.morphologyEx(cleaned, cv2.MORPH_OPEN, kernel)
            processed_masks.append(cleaned)
        
        line_mask = processed_masks[0]
        for mask in processed_masks[1:]:
            line_mask = cv2.bitwise_or(line_mask, mask)
        
        horizontal_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (7, 3))
        line_mask = cv2.morphologyEx(line_mask, cv2.MORPH_CLOSE, horizontal_kernel)
        
        # Tìm contours
        _, contours, _ = cv2.findContours(line_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if not contours:
            return False, "LOW", None
        
        # Kiểm tra main line
        main_line_center = self._get_line_center(self.latest_image, self.ROI_Y, self.ROI_H)
        if main_line_center is None:
            return False, "LOW", None
        
        # Analyze cross candidates
        roi_height, roi_width = line_mask.shape
        cross_candidates = []
        
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < 150:
                continue
                
            x, y, w, h = cv2.boundingRect(contour)
            
            # Kiểm tra tỷ lệ horizontal
            aspect_ratio = w / h if h > 0 else 0
            width_ratio = w / roi_width
            height_ratio = h / roi_height
            
            if (aspect_ratio >= self.CROSS_MIN_ASPECT_RATIO and 
                width_ratio >= self.CROSS_MIN_WIDTH_RATIO and 
                height_ratio <= self.CROSS_MAX_HEIGHT_RATIO):
                
                center_x = x + w // 2
                center_y = y + h // 2
                
                cross_candidates.append({
                    'contour': contour,
                    'area': area,
                    'center_x': center_x,
                    'center_y': center_y,
                    'width_ratio': width_ratio,
                    'height_ratio': height_ratio,
                    'aspect_ratio': aspect_ratio
                })
        
        if not cross_candidates:
            return False, "LOW", None
        
        # Chọn candidate tốt nhất
        best_candidate = None
        best_score = 0
        
        for candidate in cross_candidates:
            score = (candidate['width_ratio'] * 0.4 + 
                    min(candidate['aspect_ratio'] / 3.0, 1.0) * 0.4 +
                    (1.0 - candidate['height_ratio']) * 0.2)
            
            if score > best_score:
                best_score = score
                best_candidate = candidate
        
        if best_candidate is None:
            return False, "LOW", None
        
        # Đánh giá confidence
        confidence_level = "LOW"
        if best_score > 0.55:
            confidence_level = "HIGH"
        elif best_score > 0.25:
            confidence_level = "MEDIUM"
        
        cross_line_center = best_candidate['center_x']
        return True, confidence_level, cross_line_center

    def reset_intersection_detection(self):
        """Reset trạng thái intersection detection."""
        self.camera_intersection_detected = False
        self.camera_detection_time = 0
        self.waiting_for_lidar_confirmation = False

    def move_forward_briefly(self):
        """Di chuyển thẳng ngắn để positioning LiDAR."""
        rospy.loginfo("🚀 Moving forward briefly for better LiDAR positioning...")
        
        forward_speed = self.BASE_SPEED * 0.7
        self.robot.set_motors(forward_speed, forward_speed)
        time.sleep(1.5)
        self.robot.stop()
        
        rospy.loginfo("✅ Forward movement completed")

    # ================ RED FLAG DETECTION (Giữ nguyên) ================
    
    def detect_red_flag(self, image):
        """Phát hiện cờ đỏ - giữ nguyên từ code gốc."""
        if image is None:
            return False
            
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        mask1 = cv2.inRange(hsv, self.FLAG_RED_LOWER1, self.FLAG_RED_UPPER1)
        mask2 = cv2.inRange(hsv, self.FLAG_RED_LOWER2, self.FLAG_RED_UPPER2)
        red_mask = cv2.bitwise_or(mask1, mask2)
        
        kernel = np.ones((3,3), np.uint8)
        red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, kernel)
        red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, kernel)
        
        total_pixels = image.shape[0] * image.shape[1]
        red_pixels = cv2.countNonZero(red_mask)
        coverage_ratio = red_pixels / total_pixels
        
        if coverage_ratio > 0.05:
            rospy.loginfo(f"🔴 Red flag coverage: {coverage_ratio:.2f}")
        
        return coverage_ratio > self.FLAG_COVERAGE_THRESHOLD

    # ================ YOLO DETECTION METHODS ================
    
    def numpy_nms(self, boxes, scores, iou_threshold):
        """Non-Maximum Suppression - giữ nguyên từ code gốc."""
        x1 = np.array([b[0] for b in boxes])
        y1 = np.array([b[1] for b in boxes])
        x2 = np.array([b[2] for b in boxes])
        y2 = np.array([b[3] for b in boxes])

        areas = (x2 - x1 + 1) * (y2 - y1 + 1)
        order = scores.argsort()[::-1]

        keep = []
        while order.size > 0:
            i = order[0]
            keep.append(i)

            xx1 = np.maximum(x1[i], x1[order[1:]])
            yy1 = np.maximum(y1[i], y1[order[1:]])
            xx2 = np.minimum(x2[i], x2[order[1:]])
            yy2 = np.minimum(y2[i], y2[order[1:]])

            w = np.maximum(0.0, xx2 - xx1 + 1)
            h = np.maximum(0.0, yy2 - yy1 + 1)
            inter = w * h

            ovr = inter / (areas[i] + areas[order[1:]] - inter)
            inds = np.where(ovr <= iou_threshold)[0]
            order = order[inds + 1]

        return np.array(keep)

    def detect_with_yolo(self, image):
        """Phát hiện đối tượng bằng YOLO - giữ nguyên từ code gốc với điều chỉnh class names."""
        if self.yolo_session is None:
            return []

        original_height, original_width = image.shape[:2]

        img_resized = cv2.resize(image, self.YOLO_INPUT_SIZE)
        img_data = np.array(img_resized, dtype=np.float32) / 255.0
        img_data = np.transpose(img_data, (2, 0, 1))
        input_tensor = np.expand_dims(img_data, axis=0)

        input_name = self.yolo_session.get_inputs()[0].name
        outputs = self.yolo_session.run(None, {input_name: input_tensor})

        predictions = np.squeeze(outputs[0]).T

        scores = np.max(predictions[:, 4:], axis=1)
        predictions = predictions[scores > self.YOLO_CONF_THRESHOLD, :]
        scores = scores[scores > self.YOLO_CONF_THRESHOLD]

        if predictions.shape[0] == 0:
            return []

        class_ids = np.argmax(predictions[:, 4:], axis=1)

        x, y, w, h = predictions[:, 0], predictions[:, 1], predictions[:, 2], predictions[:, 3]
        
        x_scale = original_width / self.YOLO_INPUT_SIZE[0]
        y_scale = original_height / self.YOLO_INPUT_SIZE[1]

        x1 = (x - w / 2) * x_scale
        y1 = (y - h / 2) * y_scale
        x2 = (x + w / 2) * x_scale
        y2 = (y + h / 2) * y_scale
        
        boxes = np.column_stack((x1, y1, x2, y2)).tolist()
        
        nms_threshold = 0.45
        indices = self.numpy_nms(np.array(boxes), scores, nms_threshold)
        
        if len(indices) == 0:
            return []

        final_detections = []
        for i in indices.flatten():
            class_name = self.YOLO_CLASS_NAMES[class_ids[i]] if class_ids[i] < len(self.YOLO_CLASS_NAMES) else str(class_ids[i])
            final_detections.append({
                'class_name': class_name,
                'confidence': scores[i],
                'box': [int(x1[i]), int(y1[i]), int(x2[i]), int(y2[i])]
            })

        rospy.loginfo(f"🔍 YOLO detected {len(final_detections)} objects")
        return final_detections

    # ================ SIGN SCANNING & PROCESSING ================
    
    def scan_for_signs(self):
        """Quay để quét biển báo ở phía Tây Bắc."""
        scan_angle = self.SIGN_SCAN_ANGLES.get(self.current_direction, 0)
        
        rospy.loginfo(f"🔄 Quay {scan_angle}° từ {self.current_direction.name} để quét biển báo (nhìn về Tây Bắc)")
        
        # Thực hiện quay
        if scan_angle != 0:
            duration = abs(scan_angle) / 90.0 * self.TURN_DURATION_90_DEG
            
            if scan_angle > 0:
                self.robot.set_motors(self.TURN_SPEED, -self.TURN_SPEED)  # Quay phải
            else:
                self.robot.set_motors(-self.TURN_SPEED, self.TURN_SPEED)  # Quay trái
                
            time.sleep(duration)
            self.robot.stop()
            time.sleep(0.5)  # Ổn định
        
        # Chụp ảnh và detect
        if self.latest_image is not None:
            rospy.loginfo("📸 Capturing image for YOLO detection...")
            time.sleep(0.3)  # Đợi ổn định
            detections = self.detect_with_yolo(self.latest_image)
            
            # Log detected signs
            for det in detections:
                rospy.loginfo(f"   🔍 Found: {det['class_name']} (conf: {det['confidence']:.2f})")
            
            self.detected_signs = detections
        else:
            rospy.logwarn("⚠️ No image available for YOLO detection")
            self.detected_signs = []
        
        # Quay về vị trí ban đầu
        if scan_angle != 0:
            return_angle = -scan_angle
            duration = abs(return_angle) / 90.0 * self.TURN_DURATION_90_DEG
            
            if return_angle > 0:
                self.robot.set_motors(self.TURN_SPEED, -self.TURN_SPEED)
            else:
                self.robot.set_motors(-self.TURN_SPEED, self.TURN_SPEED)
                
            time.sleep(duration)
            self.robot.stop()
            time.sleep(0.5)
        
        rospy.loginfo(f"🔄 Quay về vị trí ban đầu, phát hiện {len(self.detected_signs)} biển báo")

    def process_sign_decision(self):
        """Xử lý quyết định dựa trên biển báo đã phát hiện."""
        
        if not self.detected_signs:
            rospy.logwarn("⚠️ Không phát hiện biển báo nào - tiếp tục hướng hiện tại")
            return self.current_direction, "no_signs"
        
        # Phân loại biển báo
        goal_signs = [d for d in self.detected_signs if d['class_name'] == self.GOAL_SIGN]
        direction_signs = [d for d in self.detected_signs if d['class_name'] in self.DIRECTION_SIGNS]
        prohibition_signs = [d for d in self.detected_signs if d['class_name'] in self.PROHIBITION_SIGNS]
        data_items = [d for d in self.detected_signs if d['class_name'] in self.DATA_ITEMS]
        
        rospy.loginfo("📊 Phân tích biển báo:")
        rospy.loginfo(f"   🎯 Đích (L): {len(goal_signs)} biển")
        rospy.loginfo(f"   📍 Hướng (N,E,S,W): {[d['class_name'] for d in direction_signs]}")
        rospy.loginfo(f"   🚫 Cấm (NN,NE,NS,NW): {[d['class_name'] for d in prohibition_signs]}")
        rospy.loginfo(f"   📄 Dữ liệu: {[d['class_name'] for d in data_items]}")
        
        # Xử lý data items (QR, math) - publish to MQTT
        for item in data_items:
            self.process_data_item(item)
        
        # QUYẾT ĐỊNH ĐIỀU HƯỚNG
        
        # Ưu tiên 1: Biển đích đến "L"
        if goal_signs:
            best_goal = max(goal_signs, key=lambda x: x['confidence'])
            rospy.loginfo(f"🎯 PHÁT HIỆN ĐÍCH 'L'! Confidence: {best_goal['confidence']:.2f}")
            return None, "goal_reached"
        
        # Ưu tiên 2: Biển hướng tuyệt đối
        if direction_signs:
            # Chọn biển có confidence cao nhất
            best_direction_sign = max(direction_signs, key=lambda x: x['confidence'])
            target_direction_label = best_direction_sign['class_name']
            target_direction = self.LABEL_TO_DIRECTION_ENUM[target_direction_label]
            
            rospy.loginfo(f"📍 Biển hướng tốt nhất: {target_direction_label} (conf: {best_direction_sign['confidence']:.2f})")
            
            # Kiểm tra biển cấm
            is_prohibited = self.check_direction_prohibited(target_direction, prohibition_signs)
            
            if is_prohibited:
                rospy.logwarn(f"🚫 Hướng {target_direction_label} bị CẤM!")
                # Tìm hướng thay thế từ các biển hướng khác
                return self.find_alternative_direction(direction_signs, prohibition_signs)
            else:
                rospy.loginfo(f"✅ Chọn hướng: {target_direction_label}")
                return target_direction, f"direction_{target_direction_label}"
        
        # Ưu tiên 3: Tiếp tục hướng hiện tại (nếu không bị cấm)
        if not self.check_direction_prohibited(self.current_direction, prohibition_signs):
            rospy.loginfo(f"➡️ Tiếp tục hướng hiện tại: {self.current_direction.name}")
            return self.current_direction, "continue_current"
        
        # Ưu tiên 4: Tìm hướng khả thi (không bị cấm)
        return self.find_safe_direction(prohibition_signs)

    def check_direction_prohibited(self, direction, prohibition_signs):
        """Kiểm tra xem một hướng có bị cấm không."""
        current_dir_code = self.current_direction.name[0]  # N, E, S, W
        target_dir_code = direction.name[0]
        
        # Tạo mã biển cấm tương ứng
        prohibition_code = current_dir_code + target_dir_code
        
        for sign in prohibition_signs:
            if sign['class_name'] == prohibition_code:
                rospy.loginfo(f"🚫 Tìm thấy biển cấm {prohibition_code} (conf: {sign['confidence']:.2f})")
                return True
                
        return False

    def find_alternative_direction(self, direction_signs, prohibition_signs):
        """Tìm hướng thay thế từ danh sách biển hướng."""
        # Sắp xếp theo confidence giảm dần
        sorted_directions = sorted(direction_signs, key=lambda x: x['confidence'], reverse=True)
        
        for sign in sorted_directions:
            direction_label = sign['class_name']
            direction = self.LABEL_TO_DIRECTION_ENUM[direction_label]
            
            if not self.check_direction_prohibited(direction, prohibition_signs):
                rospy.loginfo(f"🔄 Chọn hướng thay thế: {direction_label} (conf: {sign['confidence']:.2f})")
                return direction, f"alternative_{direction_label}"
        
        rospy.logwarn("⚠️ Không tìm thấy hướng thay thế từ biển báo")
        return self.find_safe_direction(prohibition_signs)

    def find_safe_direction(self, prohibition_signs):
        """Tìm hướng an toàn (không bị cấm) theo thứ tự ưu tiên."""
        
        # Thứ tự ưu tiên: East (ban đầu) -> North -> South -> West
        priority_directions = [
            AbsoluteDirection.EAST,
            AbsoluteDirection.NORTH, 
            AbsoluteDirection.SOUTH,
            AbsoluteDirection.WEST
        ]
        
        # Loại bỏ hướng ngược lại
        opposite_map = {
            AbsoluteDirection.NORTH: AbsoluteDirection.SOUTH,
            AbsoluteDirection.SOUTH: AbsoluteDirection.NORTH,
            AbsoluteDirection.EAST: AbsoluteDirection.WEST,
            AbsoluteDirection.WEST: AbsoluteDirection.EAST
        }
        
        opposite_direction = opposite_map[self.current_direction]
        
        for direction in priority_directions:
            if direction == opposite_direction:
                continue  # Không đi ngược lại
                
            if not self.check_direction_prohibited(direction, prohibition_signs):
                rospy.loginfo(f"🔍 Chọn hướng an toàn: {direction.name}")
                return direction, f"safe_{direction.name}"
        
        rospy.logerr("❌ Không tìm thấy hướng an toàn nào! DEAD END!")
        return None, "dead_end"

    def process_data_item(self, item):
        """Xử lý QR code hoặc math problem và publish qua MQTT."""
        try:
            data_payload = {
                'timestamp': rospy.get_time(),
                'type': item['class_name'],
                'confidence': item['confidence'],
                'position': {
                    'current_direction': self.current_direction.name,
                    'bbox': item['box']
                }
            }
            
            if item['class_name'] == 'qr_code':
                # Thử decode QR code từ image
                qr_data = self.decode_qr_from_bbox(item['box'])
                data_payload['qr_data'] = qr_data
                rospy.loginfo(f"📱 QR Code: {qr_data}")
                
            elif item['class_name'] == 'math_problem':
                # Giả lập giải toán
                data_payload['math_result'] = "2+2=4"  # Placeholder
                rospy.loginfo("🔢 Math problem detected")
            
            # Publish to MQTT
            self.mqtt_client.publish(
                self.MQTT_DATA_TOPIC, 
                json.dumps(data_payload)
            )
            
        except Exception as e:
            rospy.logwarn(f"Error processing data item: {e}")

    def decode_qr_from_bbox(self, bbox):
        """Thử decode QR code từ bounding box."""
        if self.latest_image is None:
            return None
            
        try:
            x1, y1, x2, y2 = bbox
            qr_region = self.latest_image[y1:y2, x1:x2]
            decoded_objects = decode(qr_region)
            
            if decoded_objects:
                return decoded_objects[0].data.decode('utf-8')
                
        except Exception as e:
            rospy.logwarn(f"QR decode error: {e}")
            
        return None

    # ================ DIRECTION & TURNING ================
    
    def turn_to_direction(self, target_direction):
        """Quay robot đến hướng mục tiêu."""
        if target_direction == self.current_direction:
            rospy.loginfo("➡️ Tiếp tục đi thẳng")
            return True
        
        # Tính góc cần quay
        current_angle = self.current_direction.value * 90
        target_angle = target_direction.value * 90
        
        # Tính góc quay ngắn nhất
        angle_diff = target_angle - current_angle
        
        # Chuẩn hóa góc
        if angle_diff > 180:
            angle_diff -= 360
        elif angle_diff < -180:
            angle_diff += 360
        
        rospy.loginfo(f"🔄 Quay {angle_diff}° từ {self.current_direction.name} đến {target_direction.name}")
        
        # Thực hiện quay
        if angle_diff != 0:
            duration = abs(angle_diff) / 90.0 * self.TURN_DURATION_90_DEG
            
            if angle_diff > 0:
                self.robot.set_motors(self.TURN_SPEED, -self.TURN_SPEED)  # Quay phải
            else:
                self.robot.set_motors(-self.TURN_SPEED, self.TURN_SPEED)  # Quay trái
                
            time.sleep(duration)
            self.robot.stop()
            time.sleep(0.5)
        
        # Cập nhật hướng hiện tại
        self.current_direction = target_direction
        rospy.loginfo(f"✅ Đã quay xong, hướng hiện tại: {self.current_direction.name}")
        
        return True

    # ================ VIDEO & DEBUG ================
    
    def draw_debug_info(self, image):
        """Vẽ thông tin debug lên image."""
        if image is None:
            return None
        
        debug_frame = image.copy()
        
        # Vẽ ROI
        cv2.rectangle(debug_frame, (0, self.ROI_Y), (self.WIDTH-1, self.ROI_Y + self.ROI_H), (0, 255, 0), 1)
        cv2.rectangle(debug_frame, (0, self.LOOKAHEAD_ROI_Y), (self.WIDTH-1, self.LOOKAHEAD_ROI_Y + self.LOOKAHEAD_ROI_H), (0, 255, 255), 1)
        
        # Thông tin trạng thái
        state_text = f"State: {self.current_state.name if self.current_state else 'None'}"
        cv2.putText(debug_frame, state_text, (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        # Thông tin hướng
        dir_text = f"Direction: {self.current_direction.name}"
        cv2.putText(debug_frame, dir_text, (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        # Vẽ line center nếu đang bám line
        if self.current_state == RobotState.DRIVING_STRAIGHT:
            line_center = self._get_line_center(image, self.ROI_Y, self.ROI_H)
            if line_center is not None:
                cv2.circle(debug_frame, (line_center, self.ROI_Y + self.ROI_H // 2), 5, (0, 0, 255), -1)
        
        return debug_frame

    def _record_frame(self):
        """Ghi frame vào video."""
        if self.video_writer and self.latest_image is not None:
            debug_frame = self.draw_debug_info(self.latest_image)
            if debug_frame is not None:
                self.video_writer.write(debug_frame)

    # ================ MAIN CONTROL LOOP ================
    
    def run(self):
        """Vòng lặp chính của robot."""
        rospy.loginfo("🚀 Bắt đầu YOLO Sign Follower...")
        rospy.loginfo("Đợi 3 giây..."); time.sleep(3); rospy.loginfo("Hành trình bắt đầu!")
        
        self.detector.start_scanning()
        rate = rospy.Rate(20)
        
        while not rospy.is_shutdown() and not self.goal_reached:
            
            # Ghi video
            self._record_frame()
            
            current_time = rospy.get_time()
            elapsed_in_state = current_time - self.state_change_time
            
            # ==================== STATE MACHINE ====================
            
            if self.current_state == RobotState.WAITING_FOR_RED_FLAG:
                
                # Chờ cờ đỏ để bắt đầu
                if self.latest_image is not None:
                    if not self.initial_red_flag_detected:
                        if self.detect_red_flag(self.latest_image):
                            rospy.loginfo("🔴 PHÁT HIỆN CỜ ĐỎ! Bắt đầu di chuyển...")
                            self.initial_red_flag_detected = True
                            self.robot_started = True
                            self._set_state(RobotState.DRIVING_STRAIGHT)
                        else:
                            self.robot.stop()
                
            elif self.current_state == RobotState.DRIVING_STRAIGHT:
                
                # Bám theo line và kiểm tra giao lộ
                if self.latest_image is not None:
                    line_center = self._get_line_center(self.latest_image, self.ROI_Y, self.ROI_H)
                    
                    if line_center is not None:
                        self.correct_course(line_center)
                        
                        # Kiểm tra intersection
                        if self.check_camera_lidar_intersection():
                            rospy.loginfo("🛑 Phát hiện giao lộ!")
                            self.robot.stop()
                            self._set_state(RobotState.APPROACHING_INTERSECTION)
                            
                    else:
                        rospy.logwarn("⚠️ Mất line!")
                        self.robot.stop()
                        self._set_state(RobotState.LINE_VALIDATION)
                
            elif self.current_state == RobotState.LINE_VALIDATION:
                
                # Validate line position
                if self._is_line_in_valid_range(self.latest_image):
                    rospy.loginfo("✅ Line validation passed")
                    self._set_state(RobotState.DRIVING_STRAIGHT)
                else:
                    self.line_validation_attempts += 1
                    
                    if self.line_validation_attempts >= self.LINE_VALIDATION_ATTEMPTS:
                        rospy.logerr("❌ Line validation failed too many times")
                        self._set_state(RobotState.DEAD_END)
                    elif elapsed_in_state > self.LINE_VALIDATION_TIMEOUT:
                        rospy.logwarn("⏰ Line validation timeout")
                        self._set_state(RobotState.DRIVING_STRAIGHT)
                
            elif self.current_state == RobotState.APPROACHING_INTERSECTION:
                
                # Dừng và chuẩn bị quét biển báo
                self.robot.stop()
                time.sleep(1.0)  # Ổn định
                self._set_state(RobotState.SCANNING_SIGNS)
                
            elif self.current_state == RobotState.SCANNING_SIGNS:
                
                # Quét biển báo bằng YOLO
                self.scan_for_signs()
                self._set_state(RobotState.PROCESSING_DECISION)
                
            elif self.current_state == RobotState.PROCESSING_DECISION:
                
                # Xử lý quyết định từ biển báo
                target_direction, decision_reason = self.process_sign_decision()
                
                rospy.loginfo(f"🧠 Quyết định: {decision_reason}")
                
                if decision_reason == "goal_reached":
                    self._set_state(RobotState.GOAL_REACHED)
                elif decision_reason == "dead_end":
                    self._set_state(RobotState.DEAD_END)
                elif target_direction is None:
                    rospy.logerr("❌ Không thể xác định hướng đi!")
                    self._set_state(RobotState.DEAD_END)
                else:
                    self.current_decision = target_direction
                    if target_direction != self.current_direction:
                        self._set_state(RobotState.TURNING)
                    else:
                        self._set_state(RobotState.DRIVING_STRAIGHT)
                
            elif self.current_state == RobotState.TURNING:
                
                # Thực hiện quay đến hướng mới
                if self.turn_to_direction(self.current_decision):
                    self._set_state(RobotState.DRIVING_STRAIGHT)
                else:
                    rospy.logerr("❌ Lỗi khi quay robot!")
                    self._set_state(RobotState.DEAD_END)
                
            elif self.current_state == RobotState.GOAL_REACHED:
                
                # Đã đến đích
                self.robot.stop()
                rospy.loginfo("🎉 ĐÃ ĐẾN ĐÍCH! Phát hiện biển 'L'")
                self.goal_reached = True
                
            elif self.current_state == RobotState.DEAD_END:
                
                # Không tìm được đường đi
                self.robot.stop()
                rospy.logerr("❌ DEAD END! Không thể tiếp tục")
                break
            
            rate.sleep()
        
        # Cleanup
        self.cleanup()

    def cleanup(self):
        """Dọn dẹp tài nguyên."""
        rospy.loginfo("🧹 Dừng robot và giải phóng tài nguyên...")
        
        self.robot.stop()
        self.detector.stop_scanning()
        
        if self.mqtt_client:
            self.mqtt_client.loop_stop()
            self.mqtt_client.disconnect()
        
        if self.video_writer:
            self.video_writer.release()
        
        rospy.loginfo("✅ Đã giải phóng tài nguyên. Chương trình kết thúc.")


def main():
    """Hàm main."""
    rospy.init_node('yolo_sign_follower_node', anonymous=True)
    
    try:
        controller = YOLOSignFollower()
        controller.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Node bị ngắt")
    except Exception as e:
        rospy.logerr(f"Lỗi: {e}")


if __name__ == '__main__':
    main()