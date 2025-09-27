#!/usr/bin/env python3
"""
Test script để verify logic di chuyển sau camera detection
"""

import cv2
import numpy as np
import time
import sys
import os

# Add current directory to path
sys.path.append(os.path.dirname(__file__))

# Mock modules for testing
class MockRospy:
    def loginfo(self, msg): print(f"[INFO] {msg}")
    def logwarn(self, msg): print(f"[WARN] {msg}")
    def get_time(self): return time.time()
    def is_shutdown(self): return False

# Mock Robot
class MockRobot:
    def __init__(self):
        self.left_speed = 0
        self.right_speed = 0
        
    def set_motors(self, left, right):
        self.left_speed = left
        self.right_speed = right
        print(f"🚗 Motors set: Left={left:.3f}, Right={right:.3f}")
        
    def stop(self):
        self.left_speed = 0
        self.right_speed = 0
        print("🛑 Motors stopped")

# Mock jetbot module
class MockJetbot:
    Robot = MockRobot

sys.modules['rospy'] = MockRospy()
sys.modules['jetbot'] = MockJetbot()

# Import the main controller
from ros_lidar_follower import JetBotController

def test_forward_movement_logic():
    """Test camera detection với forward movement"""
    print("=" * 60)
    print("TEST: Camera Detection + Forward Movement Logic")
    print("=" * 60)
    
    # Create controller
    controller = JetBotController()
    controller.robot = MockRobot()  # Replace with mock
    
    # Create test image with intersection
    height, width = 480, 640
    test_image = np.zeros((height, width, 3), dtype=np.uint8)
    
    # Draw main line (vertical)
    cv2.line(test_image, (width//2, 0), (width//2, height), (0, 255, 0), 15)
    
    # Draw cross line (horizontal) - intersection
    cross_y = height // 2
    cv2.line(test_image, (0, cross_y), (width, cross_y), (0, 255, 0), 12)
    
    controller.latest_image = test_image
    
    print("\n1. Gọi check_camera_lidar_intersection() lần đầu...")
    result1 = controller.check_camera_lidar_intersection()
    
    print(f"   Kết quả: {result1}")
    print(f"   camera_intersection_detected: {controller.camera_intersection_detected}")
    print(f"   waiting_for_lidar_confirmation: {controller.waiting_for_lidar_confirmation}")
    
    print("\n2. Gọi check_camera_lidar_intersection() lần thứ 2 (waiting for LiDAR)...")
    result2 = controller.check_camera_lidar_intersection()
    
    print(f"   Kết quả: {result2}")
    print(f"   Đang chờ LiDAR confirmation...")
    
    print("\n3. Test timeout logic...")
    # Fast forward time để test timeout
    controller.camera_detection_time = time.time() - 10  # 10 seconds ago
    result3 = controller.check_camera_lidar_intersection()
    
    print(f"   Kết quả sau timeout: {result3}")
    print(f"   camera_intersection_detected: {controller.camera_intersection_detected}")
    print(f"   waiting_for_lidar_confirmation: {controller.waiting_for_lidar_confirmation}")

def test_move_forward_briefly():
    """Test move_forward_briefly method riêng"""
    print("=" * 60)
    print("TEST: move_forward_briefly() Method")
    print("=" * 60)
    
    controller = JetBotController()
    controller.robot = MockRobot()
    
    print("Gọi move_forward_briefly()...")
    start_time = time.time()
    controller.move_forward_briefly()
    end_time = time.time()
    
    print(f"Duration: {end_time - start_time:.2f} seconds")
    print(f"BASE_SPEED: {controller.BASE_SPEED}")
    print(f"Forward speed used: {controller.BASE_SPEED * 0.7:.3f}")

if __name__ == "__main__":
    print("🧪 Testing Forward Movement After Camera Detection")
    
    test_move_forward_briefly()
    print("\n" + "-" * 60 + "\n")
    test_forward_movement_logic()
    
    print("\n✅ Tests completed!")