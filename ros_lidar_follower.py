#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
import time
import os
import json
import math
from enum import Enum
import requests

from jetbot import Robot
import onnxruntime as ort
from pyzbar.pyzbar import decode
import paho.mqtt.client as mqtt
from sensor_msgs.msg import LaserScan, Image
from opposite_detector import SimpleOppositeDetector

from map_navigator import MapNavigator

class RobotState(Enum):
    DRIVING_STRAIGHT = 1
    LINE_VALIDATION = 1.5
    APPROACHING_INTERSECTION = 2
    HANDLING_EVENT = 3
    LEAVING_INTERSECTION = 4
    REACQUIRING_LINE = 5
    DEAD_END = 6
    GOAL_REACHED = 7

class Direction(Enum):
    NORTH, EAST, SOUTH, WEST = 0, 1, 2, 3

class JetBotController:
    def __init__(self):
        rospy.loginfo("Đang khởi tạo JetBot Event-Driven Controller...")
        self.setup_parameters()
        self.initialize_hardware()
        self.initialize_yolo()
        self.initialize_mqtt()

        self.video_writer = None
        self.initialize_video_writer()

        self.navigator = MapNavigator(self.MAP_FILE_PATH)
        self.current_node_id = self.navigator.start_node
        self.target_node_id = None
        self.planned_path = None
        self.banned_edges = []
        self.plan_initial_route()

        self.latest_scan = None
        self.latest_image = None
        self.detector = SimpleOppositeDetector()
        
        # Initialize angle analyzer for line/lidar comparison
        self.angle_analysis_enabled = True
        self.last_angle_analysis_time = 0
        self.angle_analysis_interval = 2.0  # Analyze every 2 seconds
        
        rospy.Subscriber('/scan', LaserScan, self.detector.callback)
        rospy.Subscriber('/csi_cam_0/image_raw', Image, self.camera_callback)
        rospy.loginfo("Đã đăng ký vào các topic /scan và /csi_cam_0/image_raw.")
        self.state_change_time = rospy.get_time()
        self.line_validation_attempts = 0  # Counter cho LINE_VALIDATION state
        self._set_state(RobotState.DRIVING_STRAIGHT, initial=True)
        rospy.loginfo("Khởi tạo hoàn tất. Sẵn sàng hoạt động.")

    def plan_initial_route(self): 
        """Lập kế hoạch đường đi ban đầu từ điểm xuất phát đến đích."""
        rospy.loginfo(f"Đang lập kế hoạch từ node {self.navigator.start_node} đến {self.navigator.end_node}...")
        self.planned_path = self.navigator.find_path(
            self.navigator.start_node, 
            self.navigator.end_node,
            self.banned_edges
        )
        if self.planned_path and len(self.planned_path) > 1:
            self.target_node_id = self.planned_path[1]
            rospy.loginfo(f"Đã tìm thấy đường đi: {self.planned_path}. Đích đến đầu tiên: {self.target_node_id}")
        else:
            rospy.logerr("Không tìm thấy đường đi hoặc đường đi quá ngắn!")
            self._set_state(RobotState.DEAD_END)

    def initialize_video_writer(self):
        """Khởi tạo đối tượng VideoWriter."""
        try:
            # Kích thước video sẽ giống kích thước ảnh robot xử lý
            frame_size = (self.WIDTH, self.HEIGHT)
            self.video_writer = cv2.VideoWriter(self.VIDEO_OUTPUT_FILENAME, 
                                                self.VIDEO_FOURCC, 
                                                self.VIDEO_FPS, 
                                                frame_size)
            if self.video_writer.isOpened():
                rospy.loginfo(f"Bắt đầu ghi video vào file '{self.VIDEO_OUTPUT_FILENAME}'")
            else:
                rospy.logerr("Không thể mở file video để ghi.")
                self.video_writer = None
        except Exception as e:
            rospy.logerr(f"Lỗi khi khởi tạo VideoWriter: {e}")
            self.video_writer = None

    def draw_debug_info(self, image):
        """Vẽ các thông tin gỡ lỗi lên một khung hình."""
        if image is None:
            return None
        
        debug_frame = image.copy()
        
        # 1. Vẽ các ROI
        # ROI Chính (màu xanh lá)
        cv2.rectangle(debug_frame, (0, self.ROI_Y), (self.WIDTH-1, self.ROI_Y + self.ROI_H), (0, 255, 0), 1)
        # ROI Dự báo (màu vàng)
        cv2.rectangle(debug_frame, (0, self.LOOKAHEAD_ROI_Y), (self.WIDTH-1, self.LOOKAHEAD_ROI_Y + self.LOOKAHEAD_ROI_H), (0, 255, 255), 1)

        # 2. Vẽ trạng thái hiện tại
        state_text = f"State: {self.current_state.name if self.current_state else 'None'}"
        cv2.putText(debug_frame, state_text, (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
        
        # 3. Vẽ line và trọng tâm (nếu robot đang bám line)
        if self.current_state == RobotState.DRIVING_STRAIGHT:
            # Lấy line center của ROI Chính
            line_center = self._get_line_center(image, self.ROI_Y, self.ROI_H)
            if line_center is not None:
                # Vẽ một đường thẳng đứng màu đỏ tại vị trí trọng tâm
                cv2.line(debug_frame, (line_center, self.ROI_Y), (line_center, self.ROI_Y + self.ROI_H), (0, 0, 255), 2)

        return debug_frame

    def setup_parameters(self):
        self.WIDTH, self.HEIGHT = 300, 300
        self.BASE_SPEED = 0.16
        self.TURN_SPEED = 0.2
        self.TURN_DURATION_90_DEG = 0.8
        self.ROI_Y = int(self.HEIGHT * 0.85)
        self.ROI_H = int(self.HEIGHT * 0.15)
        self.ROI_CENTER_WIDTH_PERCENT = 0.5
        self.LOOKAHEAD_ROI_Y = int(self.HEIGHT * 0.60) # Vị trí Y cao hơn
        self.LOOKAHEAD_ROI_H = int(self.HEIGHT * 0.15) # Chiều cao tương tự

        self.CORRECTION_GAIN = 0.5
        self.SAFE_ZONE_PERCENT = 0.3
        self.LINE_COLOR_LOWER = np.array([0, 0, 0])
        self.LINE_COLOR_UPPER = np.array([180, 255, 75])
        self.INTERSECTION_CLEARANCE_DURATION = 1.5
        self.INTERSECTION_APPROACH_DURATION = 0.5
        self.LINE_REACQUIRE_TIMEOUT = 3.0
        self.SCAN_PIXEL_THRESHOLD = 100
        self.YOLO_MODEL_PATH = "models/best.onnx"
        self.YOLO_CONF_THRESHOLD = 0.6
        self.YOLO_INPUT_SIZE = (640, 640)
        self.YOLO_CLASS_NAMES = ['N', 'E', 'W', 'S', 'NN', 'NE', 'NW', 'NS', 'math']
        self.PRESCRIPTIVE_SIGNS = {'N', 'E', 'W', 'S'}
        self.PROHIBITIVE_SIGNS = {'NN', 'NE', 'NW', 'NS'}
        self.DATA_ITEMS = {'qr_code', 'math_problem'}
        self.MQTT_BROKER = "localhost"; self.MQTT_PORT = 1883
        self.MQTT_DATA_TOPIC = "jetbot/corrected_event_data"
        self.current_state = None
        self.DIRECTIONS = [Direction.NORTH, Direction.EAST, Direction.SOUTH, Direction.WEST]
        self.current_direction_index = 1
        self.ANGLE_TO_FACE_SIGN_MAP = {d: a for d, a in zip(self.DIRECTIONS, [45, -45, -135, 135])}
        self.MAX_CORRECTION_ADJ = 0.12
        self.MAP_FILE_PATH = "map.json"
        self.LABEL_TO_DIRECTION_ENUM = {'N': Direction.NORTH, 'E': Direction.EAST, 'S': Direction.SOUTH, 'W': Direction.WEST}
        self.VIDEO_OUTPUT_FILENAME = 'jetbot_run.avi'
        self.VIDEO_FPS = 20  # Nên khớp với rospy.Rate của bạn
        # Codec 'MJPG' rất phổ biến và tương thích tốt
        self.VIDEO_FOURCC = cv2.VideoWriter_fourcc(*'MJPG')
        
        # Parameters cho LINE_VALIDATION state
        self.LINE_VALIDATION_TIMEOUT = 2.0  # Thời gian tối đa để validate line position
        self.LINE_CENTER_TOLERANCE = 0.2    # Tỷ lệ cho phép line lệch khỏi center (20% width)
        self.LINE_VALIDATION_ATTEMPTS = 8   # Số lần thử validate tối đa

    def initialize_hardware(self):
        try:
            self.robot = Robot()
            rospy.loginfo("Phần cứng JetBot (động cơ) đã được khởi tạo.")
        except Exception as e:
            rospy.logwarn(f"Không tìm thấy phần cứng JetBot, sử dụng Mock object. Lỗi: {e}")
            from unittest.mock import Mock
            self.robot = Mock()

    def initialize_yolo(self):
        """Tải mô hình YOLO vào ONNX Runtime."""
        try:
            self.yolo_session = ort.InferenceSession(self.YOLO_MODEL_PATH, providers=['CUDAExecutionProvider', 'CPUExecutionProvider'])
            rospy.loginfo("Tải mô hình YOLO thành công.")
        except Exception as e:
            rospy.logerr(f"Không thể tải mô hình YOLO từ '{self.YOLO_MODEL_PATH}'. Lỗi: {e}")
            self.yolo_session = None

    def numpy_nms(self, boxes, scores, iou_threshold):
        """
        Thực hiện Non-Maximum Suppression (NMS) bằng NumPy.
        :param boxes: list các bounding box, mỗi box là [x1, y1, x2, y2]
        :param scores: list các điểm tin cậy tương ứng
        :param iou_threshold: ngưỡng IoU để loại bỏ các box trùng lặp
        :return: list các chỉ số (indices) của các box được giữ lại
        """
        # Chuyển đổi sang NumPy array để tính toán vector hóa
        x1 = np.array([b[0] for b in boxes])
        y1 = np.array([b[1] for b in boxes])
        x2 = np.array([b[2] for b in boxes])
        y2 = np.array([b[3] for b in boxes])

        areas = (x2 - x1 + 1) * (y2 - y1 + 1)
        # Sắp xếp các box theo điểm tin cậy giảm dần
        order = scores.argsort()[::-1]

        keep = []
        while order.size > 0:
            i = order[0]
            keep.append(i)
            
            # Tính toán IoU (Intersection over Union)
            xx1 = np.maximum(x1[i], x1[order[1:]])
            yy1 = np.maximum(y1[i], y1[order[1:]])
            xx2 = np.minimum(x2[i], x2[order[1:]])
            yy2 = np.minimum(y2[i], y2[order[1:]])

            w = np.maximum(0.0, xx2 - xx1 + 1)
            h = np.maximum(0.0, yy2 - yy1 + 1)
            intersection = w * h
            
            iou = intersection / (areas[i] + areas[order[1:]] - intersection)

            # Giữ lại các box có IoU nhỏ hơn ngưỡng
            inds = np.where(iou <= iou_threshold)[0]
            order = order[inds + 1]

        return np.array(keep)

    def detect_with_yolo(self, image):
        """
        Thực hiện nhận diện đối tượng bằng YOLOv8 và hậu xử lý kết quả đúng cách.
        """
        if self.yolo_session is None: return []

        original_height, original_width = image.shape[:2]

        img_resized = cv2.resize(image, self.YOLO_INPUT_SIZE)
        img_data = np.array(img_resized, dtype=np.float32) / 255.0
        img_data = np.transpose(img_data, (2, 0, 1))  # HWC to CHW
        input_tensor = np.expand_dims(img_data, axis=0)  # Add batch dimension

        input_name = self.yolo_session.get_inputs()[0].name
        outputs = self.yolo_session.run(None, {input_name: input_tensor})

        # Lấy output thô, output của YOLOv8 thường có shape (1, 84, 8400) hoặc tương tự
        # Chúng ta cần transpose nó thành (1, 8400, 84) để dễ xử lý
        predictions = np.squeeze(outputs[0]).T

        # Lọc các box có điểm tin cậy (objectness score) thấp
        # Cột 4 trong predictions là điểm tin cậy tổng thể của box
        scores = np.max(predictions[:, 4:], axis=1)
        predictions = predictions[scores > self.YOLO_CONF_THRESHOLD, :]
        scores = scores[scores > self.YOLO_CONF_THRESHOLD]

        if predictions.shape[0] == 0:
            rospy.loginfo("YOLO không phát hiện đối tượng nào vượt ngưỡng tin cậy.")
            return []

        # Lấy class_id có điểm cao nhất
        class_ids = np.argmax(predictions[:, 4:], axis=1)

        # Lấy tọa độ box và chuyển đổi về ảnh gốc
        x, y, w, h = predictions[:, 0], predictions[:, 1], predictions[:, 2], predictions[:, 3]
        
        # Tính toán tỷ lệ scale để chuyển đổi tọa độ
        x_scale = original_width / self.YOLO_INPUT_SIZE[0]
        y_scale = original_height / self.YOLO_INPUT_SIZE[1]

        # Chuyển từ [center_x, center_y, width, height] sang [x1, y1, x2, y2]
        x1 = (x - w / 2) * x_scale
        y1 = (y - h / 2) * y_scale
        x2 = (x + w / 2) * x_scale
        y2 = (y + h / 2) * y_scale
        
        # Chuyển thành list các box và scores
        boxes = np.column_stack((x1, y1, x2, y2)).tolist()
        
        # 4. Thực hiện Non-Maximum Suppression (NMS)
        # Đây là một bước cực kỳ quan trọng để loại bỏ các box trùng lặp
        # OpenCV cung cấp một hàm NMS hiệu quả
        nms_threshold = 0.45 # Ngưỡng IOU để loại bỏ box
        indices = self.numpy_nms(np.array(boxes), scores, nms_threshold)
        
        if len(indices) == 0:
            rospy.loginfo("YOLO: Sau NMS, không còn đối tượng nào.")
            return []

        # 5. Tạo danh sách kết quả cuối cùng
        final_detections = []
        for i in indices.flatten():
            final_detections.append({
                'class_name': self.YOLO_CLASS_NAMES[class_ids[i]],
                'confidence': float(scores[i]),
                'box': [int(coord) for coord in boxes[i]] # Chuyển tọa độ sang int
            })

        rospy.loginfo(f"YOLO đã phát hiện {len(final_detections)} đối tượng cuối cùng.")
        return final_detections

    def initialize_mqtt(self):
        self.mqtt_client = mqtt.Client()
        def on_connect(client, userdata, flags, rc): rospy.loginfo(f"Kết nối MQTT: {'Thành công' if rc == 0 else 'Thất bại'}")
        self.mqtt_client.on_connect = on_connect
        try:
            self.mqtt_client.connect(self.MQTT_BROKER, self.MQTT_PORT, 60)
            self.mqtt_client.loop_start()
        except Exception as e: rospy.logerr(f"Không thể kết nối MQTT: {e}")
    
    def _set_state(self, new_state, initial=False):
        if self.current_state != new_state:
            if not initial: rospy.loginfo(f"Chuyển trạng thái: {self.current_state.name if self.current_state else 'None'} -> {new_state.name}")
            
            # Reset line validation counter khi rời khỏi LINE_VALIDATION state
            if self.current_state == RobotState.LINE_VALIDATION and new_state != RobotState.LINE_VALIDATION:
                self.line_validation_attempts = 0
                
            self.current_state = new_state
            self.state_change_time = rospy.get_time()

    def camera_callback(self, image_msg):
        try:
            if image_msg.encoding.endswith('compressed'):
                np_arr = np.frombuffer(image_msg.data, np.uint8)
                cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            else:
                cv_image = np.frombuffer(image_msg.data, dtype=np.uint8).reshape(image_msg.height, image_msg.width, -1)
            if 'rgb' in image_msg.encoding: cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)
            self.latest_image = cv2.resize(cv_image, (self.WIDTH, self.HEIGHT))
        except Exception as e: rospy.logerr(f"Lỗi chuyển đổi ảnh: {e}")

    def run(self):
        rospy.loginfo("Bắt đầu vòng lặp. Đợi 3 giây..."); time.sleep(3); rospy.loginfo("Hành trình bắt đầu!")
        self.detector.start_scanning()
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            # ===================================================================
            # TRẠNG THÁI 1: ĐANG BÁM LINE (DRIVING_STRAIGHT)
            # ===================================================================
            if self.current_state == RobotState.DRIVING_STRAIGHT:
                if self.latest_image is None:
                    rospy.logwarn_throttle(5, "Đang chờ dữ liệu hình ảnh từ topic camera...")
                    self.robot.stop()
                    rate.sleep()
                    continue

                # --- BƯỚC 1: KIỂM TRA TÍN HIỆU ƯU TIÊN CAO (LiDAR) ---
                # Đây là tín hiệu đáng tin cậy nhất, nếu nó kích hoạt, xử lý ngay.
                if self.detector.process_detection():
                    rospy.loginfo("SỰ KIỆN (LiDAR): Phát hiện giao lộ. Dừng ngay lập tức.")
                    self.robot.stop()
                    time.sleep(0.5) # Chờ robot dừng hẳn

                    # Cập nhật vị trí hiện tại (đã đến đích) và xử lý
                    self.current_node_id = self.target_node_id
                    rospy.loginfo(f"==> ĐÃ ĐẾN node {self.current_node_id}.")

                    if self.current_node_id == self.navigator.end_node:
                        rospy.loginfo("ĐÃ ĐẾN ĐÍCH CUỐI CÙNG!")
                        self._set_state(RobotState.GOAL_REACHED)
                    else:
                        self._set_state(RobotState.HANDLING_EVENT)
                        self.handle_intersection()
                    continue # Bắt đầu vòng lặp mới với trạng thái mới

                # --- BƯỚC 2: LOGIC "NHÌN XA HƠN" VỚI ROI DỰ BÁO ---
                # Nếu LiDAR im lặng, kiểm tra xem vạch kẻ có sắp biến mất ở phía xa không.
                lookahead_line_center = self._get_line_center(self.latest_image, self.LOOKAHEAD_ROI_Y, self.LOOKAHEAD_ROI_H)

                if lookahead_line_center is None:
                    rospy.logwarn("SỰ KIỆN (Dự báo): Vạch kẻ đường biến mất ở phía xa. Chuẩn bị vào giao lộ.")
                    # Hành động phòng ngừa: chuyển sang trạng thái đi thẳng vào giao lộ.
                    self._set_state(RobotState.APPROACHING_INTERSECTION)
                    continue # Bắt đầu vòng lặp mới với trạng thái mới

                # --- BƯỚC 3: BÁM LINE BÌNH THƯỜNG (NẾU PHÍA TRƯỚC AN TOÀN) ---
                # Chỉ khi cả LiDAR và ROI Dự báo đều ổn, ta mới thực hiện bám line.
                execution_line_center = self._get_line_center(self.latest_image, self.ROI_Y, self.ROI_H)

                if execution_line_center is not None:
                    # Kiểm tra xem line có nằm trong khoảng hợp lệ không trước khi bám
                    if not self._is_line_in_valid_range(self.latest_image):
                        rospy.logwarn("SỰ KIỆN: Line position không hợp lệ, chuyển sang LINE_VALIDATION để kiểm tra.")
                        self.line_validation_attempts = 0  # Reset counter
                        self._set_state(RobotState.LINE_VALIDATION)
                        continue
                    
                    # An toàn để bám line, vì chúng ta biết phía trước không có giao lộ đột ngột.
                    self.correct_course(execution_line_center)
                    
                    # Phân tích và in góc line (nếu được bật)
                    if self.angle_analysis_enabled:
                        self.analyze_and_print_line_angles()
                else:
                    # Trường hợp hiếm: ROI xa thấy line nhưng ROI gần lại không. Dừng lại cho an toàn.
                    rospy.logwarn("Trạng thái không nhất quán: ROI xa thấy line, ROI gần không thấy. Tạm dừng an toàn.")
                    self.robot.stop()

            # ===================================================================
            # TRẠNG THÁI 1.5: KIỂM TRA VÀ XÁC THỰC VỊ TRÍ LINE (LINE_VALIDATION)
            # ===================================================================
            elif self.current_state == RobotState.LINE_VALIDATION:
                if self.latest_image is None:
                    rospy.logwarn("LINE_VALIDATION: Chờ dữ liệu camera...")
                    self.robot.stop()
                    rate.sleep()
                    continue

                # Kiểm tra line có nằm trong khoảng hợp lệ không
                if self._is_line_in_valid_range(self.latest_image):
                    rospy.loginfo("LINE_VALIDATION: Line position hợp lệ, tiếp tục bám line.")
                    self._set_state(RobotState.DRIVING_STRAIGHT)
                    continue
                else:
                    # Line không hợp lệ, thử điều chỉnh
                    self.line_validation_attempts += 1
                    rospy.logwarn(f"LINE_VALIDATION: Line không hợp lệ, lần thử {self.line_validation_attempts}/{self.LINE_VALIDATION_ATTEMPTS}")
                    
                    if self.line_validation_attempts >= self.LINE_VALIDATION_ATTEMPTS:
                        rospy.logerr("LINE_VALIDATION: Đã thử tối đa, chuyển sang tìm kiếm line mới.")
                        self._set_state(RobotState.REACQUIRING_LINE)
                        continue
                    
                    # Thử điều chỉnh nhẹ để tìm lại line
                    line_center = self._get_line_center(self.latest_image, self.ROI_Y, self.ROI_H)
                    if line_center is not None:
                        error = line_center - (self.WIDTH / 2)
                        if abs(error) > 0:
                            # Điều chỉnh nhẹ về phía line
                            adj = np.clip(error / (self.WIDTH / 2) * 0.3, -0.1, 0.1)
                            self.robot.set_motors(self.BASE_SPEED * 0.5 + adj, self.BASE_SPEED * 0.5 - adj)
                        else:
                            self.robot.set_motors(self.BASE_SPEED * 0.5, self.BASE_SPEED * 0.5)
                    else:
                        self.robot.stop()
                
                # Timeout check
                if rospy.get_time() - self.state_change_time > self.LINE_VALIDATION_TIMEOUT:
                    rospy.logwarn("LINE_VALIDATION: Timeout, chuyển sang tìm kiếm line mới.")
                    self._set_state(RobotState.REACQUIRING_LINE)

            # ===================================================================
            # TRẠNG THÁI 2: ĐANG TIẾN VÀO GIAO LỘ (APPROACHING_INTERSECTION)
            # ===================================================================
            elif self.current_state == RobotState.APPROACHING_INTERSECTION:
                # Đi thẳng một đoạn ngắn để vào trung tâm giao lộ
                self.robot.set_motors(self.BASE_SPEED, self.BASE_SPEED)
                
                if rospy.get_time() - self.state_change_time > self.INTERSECTION_APPROACH_DURATION:
                    rospy.loginfo("Đã tiến vào trung tâm giao lộ. Dừng lại để xử lý.")
                    self.robot.stop(); time.sleep(0.5)

                    self.current_node_id = self.target_node_id
                    rospy.loginfo(f"==> ĐÃ ĐẾN node {self.current_node_id}.")

                    if self.current_node_id == self.navigator.end_node:
                        rospy.loginfo("ĐÃ ĐẾN ĐÍCH CUỐI CÙNG!")
                        self._set_state(RobotState.GOAL_REACHED)
                    else:
                        self._set_state(RobotState.HANDLING_EVENT)
                        self.handle_intersection()

            # ===================================================================
            # TRẠNG THÁI 3: ĐANG RỜI KHỎI GIAO LỘ (LEAVING_INTERSECTION)
            # ===================================================================
            elif self.current_state == RobotState.LEAVING_INTERSECTION:
                self.robot.set_motors(self.BASE_SPEED, self.BASE_SPEED)
                if rospy.get_time() - self.state_change_time > self.INTERSECTION_CLEARANCE_DURATION:
                    rospy.loginfo("Đã thoát khỏi khu vực giao lộ. Bắt đầu tìm kiếm line mới.")
                    self._set_state(RobotState.REACQUIRING_LINE)
            
            # ===================================================================
            # TRẠNG THÁI 4: ĐANG TÌM LẠI LINE (REACQUIRING_LINE)
            # ===================================================================
            elif self.current_state == RobotState.REACQUIRING_LINE:
                self.robot.set_motors(self.BASE_SPEED, self.BASE_SPEED)
                line_center_x = self._get_line_center(self.latest_image, self.ROI_Y, self.ROI_H)
                
                if line_center_x is not None:
                    rospy.loginfo("Đã tìm thấy line mới! Chuyển sang chế độ bám line.")
                    self._set_state(RobotState.DRIVING_STRAIGHT)
                    continue
                
                if rospy.get_time() - self.state_change_time > self.LINE_REACQUIRE_TIMEOUT:
                    rospy.logerr("Không thể tìm thấy line mới sau khi rời giao lộ. Dừng lại.")
                    self._set_state(RobotState.DEAD_END)

            # ===================================================================
            # TRẠNG THÁI KẾT THÚC (DEAD_END, GOAL_REACHED)
            # ===================================================================
            elif self.current_state == RobotState.DEAD_END:
                rospy.logwarn("Đã vào ngõ cụt hoặc gặp lỗi không thể phục hồi. Dừng hoạt động."); self.robot.stop(); break
            elif self.current_state == RobotState.GOAL_REACHED: 
                rospy.loginfo("ĐÃ HOÀN THÀNH NHIỆM VỤ. Dừng hoạt động."); self.robot.stop(); break

            if self.video_writer is not None and self.latest_image is not None:
                # Lấy ảnh gốc, vẽ thông tin lên, rồi ghi
                debug_frame = self.draw_debug_info(self.latest_image)
                if debug_frame is not None:
                    self.video_writer.write(debug_frame)

            rate.sleep()
        self.cleanup()

    def cleanup(self):
        rospy.loginfo("Dừng robot và giải phóng tài nguyên..."); self.robot.stop()
        self.detector.stop_scanning(); self.mqtt_client.loop_stop(); self.mqtt_client.disconnect()
        rospy.loginfo("Đã giải phóng tài nguyên. Chương trình kết thúc.")

    def map_absolute_to_relative(self, target_direction_label, current_robot_direction):
        """
        Chuyển đổi hướng tuyệt đối ('N', 'E', 'S', 'W') thành hành động tương đối ('straight', 'left', 'right').
        Ví dụ: robot đang hướng BẮC (NORTH), mục tiêu là đi hướng ĐÔNG (EAST) -> hành động là 'right'.
        """
        target_dir = self.LABEL_TO_DIRECTION_ENUM.get(target_direction_label)
        if target_dir is None: return None

        current_idx = current_robot_direction.value
        target_idx = target_dir.value
        
        diff = (target_idx - current_idx + 4) % 4 
        
        if diff == 0:
            return 'straight'
        elif diff == 1:
            return 'right'
        elif diff == 3: 
            return 'left'
        else: 
            return 'turn_around'
        
    def map_relative_to_absolute(self, relative_action, current_robot_direction):
        """
        Chuyển đổi hành động tương đối ('straight', 'left', 'right') thành hướng tuyệt đối ('N', 'E', 'S', 'W').
        """
        current_idx = current_robot_direction.value
        if relative_action == 'straight':
            target_idx = current_idx
        elif relative_action == 'right':
            target_idx = (current_idx + 1) % 4
        elif relative_action == 'left':
            target_idx = (current_idx - 1 + 4) % 4
        else:
            return None
        
        for label, direction in self.LABEL_TO_DIRECTION_ENUM.items():
            if direction.value == target_idx:
                return label
        return None
    
    def _get_line_center(self, image, roi_y, roi_h):
        """Kiểm tra sự tồn tại và vị trí của vạch kẻ trong một ROI cụ thể."""
        if image is None: return None
        roi = image[roi_y : roi_y + roi_h, :]
        
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        
        # Bước 1: Tạo mặt nạ màu sắc như cũ
        color_mask = cv2.inRange(hsv, self.LINE_COLOR_LOWER, self.LINE_COLOR_UPPER)
        
        # === BƯỚC 2: TẠO MẶT NẠ TẬP TRUNG (FOCUS MASK) ===
        focus_mask = np.zeros_like(color_mask)
        roi_height, roi_width = focus_mask.shape
        
        center_width = int(roi_width * self.ROI_CENTER_WIDTH_PERCENT)
        start_x = (roi_width - center_width) // 2
        end_x = start_x + center_width
        
        # Vẽ một hình chữ nhật trắng ở giữa
        cv2.rectangle(focus_mask, (start_x, 0), (end_x, roi_height), 255, -1)
        
        # === BƯỚC 3: KẾT HỢP HAI MẶT NẠ ===
        # Chỉ giữ lại những pixel trắng nào xuất hiện ở cả hai mặt nạ
        final_mask = cv2.bitwise_and(color_mask, focus_mask)

        # (Tùy chọn) Hiển thị mask để debug
        # cv2.imshow("Color Mask", color_mask)
        # cv2.imshow("Focus Mask", focus_mask)
        # cv2.imshow("Final Mask", final_mask)
        # cv2.waitKey(1)

        # Tìm contours trên mặt nạ cuối cùng đã được lọc
        _, contours, _ = cv2.findContours(final_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if not contours:
            return None
            
        c = max(contours, key=cv2.contourArea)
        
        if cv2.contourArea(c) < self.SCAN_PIXEL_THRESHOLD:
            return None

        M = cv2.moments(c)
        if M["m00"] > 0:
            # Quan trọng: Trọng tâm bây giờ được tính toán chỉ dựa trên vạch kẻ trong khu vực trung tâm
            return int(M["m10"] / M["m00"])
        return None
    
    def _is_line_in_valid_range(self, image):
        """
        Kiểm tra xem vạch kẻ có nằm trong khoảng hợp lệ không.
        Returns: 
            - True nếu line nằm trong khoảng cho phép
            - False nếu line quá lệch hoặc không tìm thấy
        """
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
    
    def calculate_line_angle_from_camera(self):
        """
        Tính góc của line từ camera dựa trên vị trí trọng tâm.
        Returns: (angle_degrees, confidence)
        """
        line_center = self._get_line_center(self.latest_image, self.ROI_Y, self.ROI_H)
        if line_center is None:
            return None, "NO_LINE_DETECTED"
        
        image_center = self.WIDTH / 2
        pixel_offset = line_center - image_center
        
        # Chuyển đổi pixel offset thành góc (sử dụng FOV 60 degrees)
        camera_fov = 60
        angle = (pixel_offset / image_center) * (camera_fov / 2)
        
        # Đánh giá confidence
        if abs(angle) < 3:
            confidence = "HIGH"
        elif abs(angle) < 10:
            confidence = "MEDIUM"
        else:
            confidence = "LOW"
        
        return angle, confidence
    
    def find_line_clusters_in_lidar(self):
        """
        Tìm các clusters có thể là line trong dữ liệu LiDAR.
        """
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
        """
        Phát hiện line cluster trong một zone.
        """
        if len(zone_ranges) == 0:
            return None
        
        # Lọc các điểm hợp lệ (tăng range để detect line xa hơn)
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
        if np.sum(weights) > 0:
            weights = weights / np.sum(weights)  # Normalize
            weighted_angle = np.average(angles, weights=weights)
        else:
            weighted_angle = np.mean(angles)
        
        # Đánh giá confidence
        if len(line_clusters) >= 3:
            confidence = "HIGH"
        elif len(line_clusters) >= 2:
            confidence = "MEDIUM"
        else:
            confidence = "LOW"
        
        return weighted_angle, confidence
    
    def analyze_and_print_line_angles(self):
        """
        Phân tích và in ra góc line từ camera và LiDAR.
        """
        current_time = rospy.get_time()
        
        # Chỉ phân tích mỗi interval seconds
        if current_time - self.last_angle_analysis_time < self.angle_analysis_interval:
            return
        
        self.last_angle_analysis_time = current_time
        
        camera_angle, camera_conf = self.calculate_line_angle_from_camera()
        lidar_angle, lidar_conf = self.calculate_line_angle_from_lidar()
        
        print("\n" + "="*60)
        print("🔍 LINE ANGLE ANALYSIS")
        print(f"⏰ Time: {current_time:.1f}s")
        print("="*60)
        
        # Camera Analysis
        print(f"📷 CAMERA:")
        if camera_angle is not None:
            print(f"   • Angle: {camera_angle:+6.2f}° ({camera_conf})")
            if camera_angle > 5:
                print(f"   • Direction: RIGHT (line to the right)")
            elif camera_angle < -5:
                print(f"   • Direction: LEFT (line to the left)")
            else:
                print(f"   • Direction: STRAIGHT (centered)")
        else:
            print(f"   • Status: {camera_conf}")
        
        # LiDAR Analysis
        print(f"📡 LIDAR:")
        if lidar_angle is not None:
            clusters = self.find_line_clusters_in_lidar()
            print(f"   • Angle: {lidar_angle:+6.2f}° ({lidar_conf})")
            print(f"   • Clusters: {len(clusters)} found")
            for i, cluster in enumerate(clusters):
                print(f"     #{i+1}: {cluster['point_count']} pts @ {cluster['center_angle']:+5.1f}°, "
                      f"dist={cluster['distance']:.2f}m")
        else:
            print(f"   • Status: {lidar_conf}")
        
        # Comparison
        if camera_angle is not None and lidar_angle is not None:
            angle_diff = abs(camera_angle - lidar_angle)
            print(f"🔄 COMPARISON:")
            print(f"   • Difference: {angle_diff:.2f}°")
            
            if angle_diff < 3:
                agreement = "EXCELLENT"
            elif angle_diff < 8:
                agreement = "GOOD"
            elif angle_diff < 15:
                agreement = "FAIR"
            else:
                agreement = "POOR"
            
            print(f"   • Agreement: {agreement}")
            
            # Combined recommendation
            if camera_conf == "HIGH" and lidar_conf == "HIGH":
                combined_angle = (camera_angle + lidar_angle) / 2
                print(f"   • Combined: {combined_angle:+6.2f}° (averaged)")
            elif camera_conf == "HIGH":
                print(f"   • Recommend: Use Camera ({camera_angle:+6.2f}°)")
            elif lidar_conf == "HIGH":
                print(f"   • Recommend: Use LiDAR ({lidar_angle:+6.2f}°)")
            else:
                combined_angle = (camera_angle + lidar_angle) / 2
                print(f"   • Combined: {combined_angle:+6.2f}° (with caution)")
        
        print("="*60 + "\n")
    
    def correct_course(self, line_center_x):
        """
        Hàm bám line an toàn với cơ chế giới hạn lực bẻ lái.
        """
        error = line_center_x - (self.WIDTH / 2)
        
        # Vẫn đi thẳng nếu sai số rất nhỏ
        if abs(error) < (self.WIDTH / 2) * self.SAFE_ZONE_PERCENT:
            self.robot.set_motors(self.BASE_SPEED, self.BASE_SPEED)
            return

        # Tính toán lực điều chỉnh
        adj = (error / (self.WIDTH / 2)) * self.CORRECTION_GAIN

        # Ngăn chặn hành vi bẻ lái quá gắt một cách tuyệt đối
        adj = np.clip(adj, -self.MAX_CORRECTION_ADJ, self.MAX_CORRECTION_ADJ)
        
        # Áp dụng lực điều chỉnh đã được giới hạn
        left_motor = self.BASE_SPEED + adj
        right_motor = self.BASE_SPEED - adj
        self.robot.set_motors(left_motor, right_motor)
        
    def handle_intersection(self):
        rospy.loginfo("\n[GIAO LỘ] Dừng lại và xử lý...")
        self.robot.stop(); time.sleep(0.5)

        current_direction = self.DIRECTIONS[self.current_direction_index]
        angle_to_sign = self.ANGLE_TO_FACE_SIGN_MAP.get(current_direction, 0)
        self.turn_robot(angle_to_sign, False)
        image_info = self.latest_image
        detections = self.detect_with_yolo(image_info)
        self.turn_robot(-angle_to_sign, False)
        
        prescriptive_cmds = {det['class_name'] for det in detections if det['class_name'] in self.PRESCRIPTIVE_SIGNS}
        prohibitive_cmds = {det['class_name'] for det in detections if det['class_name'] in self.PROHIBITIVE_SIGNS}
        data_items = [det for det in detections if det['class_name'] in self.DATA_ITEMS]

        # 2. Xử lý các mục dữ liệu (QR, Toán) và Publish
        rospy.loginfo("[STEP 2] Processing data items...")
        for item in data_items:
            if item['class_name'] == 'qr_code':
                # Code đọc QR thật
                # box = item['box']; qr_image = self.latest_image[box[1]:box[3], box[0]:box[2]]
                # decoded = decode(qr_image)
                # if decoded: qr_data = decoded[0].data.decode('utf-8'); self.publish_data(...)
                rospy.loginfo("Found QR Code. Publishing data...")
                self.publish_data({'type': 'QR_CODE', 'value': 'simulated_data_123'})

                # response = requests.post(url, json=data)

                # print(response.status_code)

            elif item['class_name'] == 'math_problem':
                rospy.loginfo("Found Math Problem. Solving and publishing...")
                self.publish_data({'type': 'MATH_PROBLEM', 'value': '2+2=4'})
        
        
        rospy.loginfo("[STEP 3] Lập kế hoạch điều hướng theo bản đồ...")
        # 3. Lập kế hoạch Điều hướng
        final_decision = None
        is_deviation = False 

        while True:
            planned_direction_label = self.navigator.get_next_direction_label(self.current_node_id, self.planned_path)
            if not planned_direction_label:
                rospy.logerr("Lỗi kế hoạch: Không tìm thấy bước tiếp theo."); self._set_state(RobotState.DEAD_END); return
            
            planned_action = self.map_absolute_to_relative(planned_direction_label, current_direction)
            rospy.loginfo(f"Kế hoạch A* đề xuất: Đi {planned_action} (hướng {planned_direction_label})")

            # Ưu tiên 1: Biển báo bắt buộc
            intended_action = None
            if 'L' in prescriptive_cmds: intended_action = 'left'
            elif 'R' in prescriptive_cmds: intended_action = 'right'
            elif 'F' in prescriptive_cmds: intended_action = 'straight'
            
            # Ưu tiên 2: Plan
            if intended_action is None:
                intended_action = planned_action
            else:
                # Nếu hành động bắt buộc khác với kế hoạch, đánh dấu là đi chệch hướng
                if intended_action != planned_action:
                    is_deviation = True
                    rospy.logwarn(f"CHỆCH HƯỚNG! Biển báo bắt buộc ({intended_action}) khác với kế hoạch ({planned_action}).")

            # 3.3. Veto bởi biển báo cấm
            is_prohibited = (intended_action == 'straight' and 'NF' in prohibitive_cmds) or \
                            (intended_action == 'right' and 'NR' in prohibitive_cmds) or \
                            (intended_action == 'left' and 'NL' in prohibitive_cmds)

            if is_prohibited:
                rospy.logwarn(f"Hành động dự định '{intended_action}' bị CẤM!")
                
                # Nếu hành động bị cấm đến từ biển báo bắt buộc -> Lỗi bản đồ
                if is_deviation:
                    rospy.logerr("LỖI BẢN ĐỒ! Biển báo bắt buộc mâu thuẫn với biển báo cấm. Không thể đi tiếp.")
                    self._set_state(RobotState.DEAD_END); return
                
                # Nếu hành động bị cấm đến từ kế hoạch A* -> Tìm đường lại
                banned_edge = (self.current_node_id, self.planned_path[self.planned_path.index(self.current_node_id) + 1])
                if banned_edge not in self.banned_edges:
                    self.banned_edges.append(banned_edge)
                
                rospy.loginfo(f"Thêm cạnh cấm {banned_edge} và tìm đường lại...")
                new_path = self.navigator.find_path(self.current_node_id, self.navigator.end_node, self.banned_edges)
                
                if new_path:
                    self.planned_path = new_path
                    rospy.loginfo(f"Đã tìm thấy đường đi mới: {self.planned_path}")
                    continue # Quay lại đầu vòng lặp để kiểm tra với kế hoạch mới
                else:
                    rospy.logerr("Không thể tìm đường đi mới sau khi gặp biển cấm.")
                    self._set_state(RobotState.DEAD_END); return
            
            final_decision = intended_action
            break 

        # 4. Thực thi quyết định
        if final_decision == 'straight': 
            rospy.loginfo("[FINAL] Decision: Go STRAIGHT.")
        elif final_decision == 'right': 
            rospy.loginfo("[FINAL] Decision: Turn RIGHT.") 
            self.turn_robot(90, True)
        elif final_decision == 'left': 
            rospy.loginfo("[FINAL] Decision: Turn LEFT.") 
            self.turn_robot(-90, True)
        else:
            rospy.logwarn("[!!!] DEAD END! No valid paths found.") 
            self._set_state(RobotState.DEAD_END)
            return
        
        # 5. Cập nhật trạng thái robot sau khi thực hiện
        # 5.1. Xác định node tiếp theo
        next_node_id = None
        if not is_deviation:
            # Nếu đi theo kế hoạch, chỉ cần lấy node tiếp theo từ path
            next_node_id = self.planned_path[self.planned_path.index(self.current_node_id) + 1]
        else:
            # Nếu chệch hướng, phải tìm node tiếp theo dựa trên hành động đã thực hiện
            
            new_robot_direction = self.DIRECTIONS[self.current_direction_index] 
            
            executed_direction_label = None
            for label, direction_enum in self.LABEL_TO_DIRECTION_ENUM.items():
                if direction_enum == new_robot_direction:
                    executed_direction_label = label 
                    break
            
            if executed_direction_label is None:
                rospy.logerr("Lỗi logic: Không thể tìm thấy label cho hướng đi mới của robot."); self._set_state(RobotState.DEAD_END) 
                return

            next_node_id = self.navigator.get_neighbor_by_direction(self.current_node_id, executed_direction_label)
            if next_node_id is None:
                 rospy.logerr("LỖI BẢN ĐỒ! Đã thực hiện rẽ nhưng không có node tương ứng."); self._set_state(RobotState.DEAD_END); return
            
            # Quan trọng: Lập kế hoạch lại từ vị trí mới
            rospy.loginfo(f"Đã đi chệch kế hoạch. Lập lại đường đi từ node mới {next_node_id}...")
            new_path = self.navigator.find_path(next_node_id, self.navigator.end_node, self.banned_edges)
            if new_path:
                self.planned_path = new_path
                rospy.loginfo(f"Đường đi mới sau khi chệch hướng: {self.planned_path}")
            else:
                rospy.logerr("Không thể tìm đường về đích từ vị trí mới."); self._set_state(RobotState.DEAD_END); return

        self.target_node_id = next_node_id
        rospy.loginfo(f"==> Đang di chuyển đến node tiếp theo: {self.target_node_id}")
        self._set_state(RobotState.LEAVING_INTERSECTION)
    
    def turn_robot(self, degrees, update_main_direction=True):
        duration = abs(degrees) / 90.0 * self.TURN_DURATION_90_DEG
        if degrees > 0: 
            self.robot.set_motors(self.TURN_SPEED, -self.TURN_SPEED)
        elif degrees < 0: 
            self.robot.set_motors(-self.TURN_SPEED, self.TURN_SPEED)
        if degrees != 0: 
            time.sleep(duration)
        self.robot.stop()
        if update_main_direction and degrees % 90 == 0 and degrees != 0:
            num_turns = round(degrees / 90)
            self.current_direction_index = (self.current_direction_index + num_turns + 4) % 4
            rospy.loginfo(f"==> Hướng đi MỚI: {self.DIRECTIONS[self.current_direction_index].name}")
        time.sleep(0.5)
    
    def _does_path_exist_in_frame(self, image):
        if image is None: return False
        roi = image[self.ROI_Y : self.ROI_Y + self.ROI_H, :]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.LINE_COLOR_LOWER, self.LINE_COLOR_UPPER)
        _img, contours, _hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        return bool(contours) and cv2.contourArea(max(contours, key=cv2.contourArea)) > self.SCAN_PIXEL_THRESHOLD
    
    def scan_for_available_paths_proactive(self):
        rospy.loginfo("[SCAN] Bắt đầu quét chủ động...")
        paths = {"straight": False, "right": False, "left": False}
        if self.latest_image is not None:
            paths["straight"] = self._does_path_exist_in_frame(self.latest_image)
        self.turn_robot(90, update_main_direction=False); time.sleep(0.5)
        if self.latest_image is not None:
            paths["right"] = self._does_path_exist_in_frame(self.latest_image)
        self.turn_robot(-180, update_main_direction=False); time.sleep(0.5)
        if self.latest_image is not None:
            paths["left"] = self._does_path_exist_in_frame(self.latest_image)
        self.turn_robot(90, update_main_direction=False)
        rospy.loginfo(f"[SCAN] Kết quả: {paths}")
        return paths

def main():
    rospy.init_node('jetbot_controller_node', anonymous=True)
    try:
        controller = JetBotController()
        controller.run()
    except rospy.ROSInterruptException: rospy.loginfo("Node đã bị ngắt.")
    except Exception as e: rospy.logerr(f"Lỗi không xác định: {e}", exc_info=True)

if __name__ == '__main__':
    main()