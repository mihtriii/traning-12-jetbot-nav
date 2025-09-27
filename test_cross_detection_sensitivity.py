#!/usr/bin/env python3
"""
Test script để kiểm tra độ nhạy của cross detection
Tạo các test cases khác nhau để đảm bảo không bỏ lỡ cross detection
"""

import cv2
import numpy as np
import time
import sys
import os

# Add current directory to path
sys.path.append(os.path.dirname(__file__))

# Mock modules
class MockRospy:
    def loginfo(self, msg): print(f"[INFO] {msg}")
    def logwarn(self, msg): print(f"[WARN] {msg}")
    def get_time(self): return time.time()
    def is_shutdown(self): return False

class MockRobot:
    def __init__(self):
        self.left_speed = 0
        self.right_speed = 0
    def set_motors(self, left, right):
        self.left_speed = left
        self.right_speed = right
    def stop(self):
        self.left_speed = 0
        self.right_speed = 0

class MockJetbot:
    Robot = MockRobot

sys.modules['rospy'] = MockRospy()
sys.modules['jetbot'] = MockJetbot()

from ros_lidar_follower import JetBotController

def create_test_image_with_cross(width=300, height=300, cross_type="normal"):
    """Tạo test image với line đen trên nền trắng"""
    # Tạo nền trắng
    image = np.ones((height, width, 3), dtype=np.uint8) * 255
    
    # Main vertical line đen (always present)
    main_line_x = width // 2
    cv2.line(image, (main_line_x, 0), (main_line_x, height), (0, 0, 0), 8)  # Đen (0,0,0)
    
    if cross_type == "normal":
        # Cross line ngang đen bình thường
        cross_y = height // 2
        cv2.line(image, (0, cross_y), (width, cross_y), (0, 0, 0), 6)  # Đen
        
    elif cross_type == "thin":
        # Cross line mỏng hơn
        cross_y = height // 2
        cv2.line(image, (0, cross_y), (width, cross_y), (0, 0, 0), 3)  # Đen
        
    elif cross_type == "short":
        # Cross line ngắn hơn
        cross_y = height // 2
        start_x = width // 4
        end_x = 3 * width // 4
        cv2.line(image, (start_x, cross_y), (end_x, cross_y), (0, 0, 0), 6)  # Đen
        
    elif cross_type == "offset":
        # Cross line lệch khỏi center
        cross_y = int(height * 0.4)  # 40% from top
        cv2.line(image, (0, cross_y), (width, cross_y), (0, 0, 0), 6)  # Đen
        
    elif cross_type == "broken":
        # Cross line bị đứt
        cross_y = height // 2
        # Left part
        cv2.line(image, (0, cross_y), (width//3, cross_y), (0, 0, 0), 6)  # Đen
        # Right part
        cv2.line(image, (2*width//3, cross_y), (width, cross_y), (0, 0, 0), 6)  # Đen
        
    elif cross_type == "noisy":
        # Cross line với nhiễu
        cross_y = height // 2
        cv2.line(image, (0, cross_y), (width, cross_y), (0, 0, 0), 6)  # Đen
        
        # Add random noise (các chấm đen nhỏ)
        for _ in range(20):
            noise_x = np.random.randint(0, width)
            noise_y = np.random.randint(0, height)
            cv2.circle(image, (noise_x, noise_y), 2, (0, 0, 0), -1)  # Đen
    
    elif cross_type == "no_cross":
        # Chỉ có main line, không có cross
        pass
        
    return image

def test_detection_sensitivity():
    """Test detection với các loại cross khác nhau"""
    print("=" * 60)
    print("TEST: Cross Detection Sensitivity")
    print("=" * 60)
    
    controller = JetBotController()
    
    test_cases = [
        ("normal", "Cross bình thường"),
        ("thin", "Cross mỏng"),  
        ("short", "Cross ngắn"),
        ("offset", "Cross lệch center"),
        ("broken", "Cross bị đứt"),
        ("noisy", "Cross với nhiễu"),
        ("no_cross", "Không có cross (negative test)")
    ]
    
    results = {}
    
    for cross_type, description in test_cases:
        print(f"\n🧪 Testing: {description}")
        
        # Tạo test image
        test_image = create_test_image_with_cross(cross_type=cross_type)
        controller.latest_image = test_image
        
        # Run detection
        detected, confidence, cross_center = controller.detect_camera_intersection()
        
        results[cross_type] = {
            'detected': detected,
            'confidence': confidence,
            'center': cross_center,
            'description': description
        }
        
        # Print result
        if detected:
            print(f"   ✅ DETECTED: {confidence} (center: {cross_center})")
        else:
            print(f"   ❌ NOT DETECTED: {confidence}")
        
        # Save debug image
        debug_image = controller.draw_debug_info(test_image)
        if debug_image is not None:
            filename = f"debug_cross_{cross_type}.jpg"
            cv2.imwrite(filename, debug_image)
            print(f"   💾 Debug image saved: {filename}")
    
    # Summary
    print("\n" + "=" * 60)
    print("SUMMARY RESULTS:")
    print("=" * 60)
    
    detection_count = 0
    expected_detections = len(test_cases) - 1  # Exclude 'no_cross'
    
    for cross_type, result in results.items():
        status = "✅ PASS" if result['detected'] else "❌ FAIL"
        if cross_type == "no_cross":
            # For negative test, we want NOT detected
            status = "✅ PASS" if not result['detected'] else "❌ FAIL (False Positive)"
        else:
            if result['detected']:
                detection_count += 1
                
        print(f"{status:12} | {cross_type:10} | {result['description']}")
    
    print(f"\n📊 Detection Rate: {detection_count}/{expected_detections} ({detection_count/expected_detections*100:.1f}%)")
    
    if detection_count >= expected_detections * 0.8:  # 80% threshold
        print("🎉 EXCELLENT! Detection sensitivity is good")
    elif detection_count >= expected_detections * 0.6:  # 60% threshold
        print("⚠️  FAIR: Detection sensitivity needs improvement")
    else:
        print("❌ POOR: Detection sensitivity is too low")
    
    return results

def test_parameter_tuning():
    """Test với các parameters khác nhau để tìm settings tốt nhất"""
    print("=" * 60)
    print("TEST: Parameter Tuning")
    print("=" * 60)
    
    controller = JetBotController()
    
    # Create challenging test case
    test_image = create_test_image_with_cross(cross_type="thin")
    controller.latest_image = test_image
    
    # Test different parameter combinations
    parameter_sets = [
        {
            'name': 'Current',
            'CROSS_MIN_ASPECT_RATIO': 1.5,
            'CROSS_MIN_WIDTH_RATIO': 0.25,
            'CROSS_MAX_HEIGHT_RATIO': 0.9
        },
        {
            'name': 'Very Sensitive',
            'CROSS_MIN_ASPECT_RATIO': 1.0,
            'CROSS_MIN_WIDTH_RATIO': 0.15,
            'CROSS_MAX_HEIGHT_RATIO': 0.95
        },
        {
            'name': 'Conservative',
            'CROSS_MIN_ASPECT_RATIO': 2.0,
            'CROSS_MIN_WIDTH_RATIO': 0.35,
            'CROSS_MAX_HEIGHT_RATIO': 0.7
        }
    ]
    
    for params in parameter_sets:
        print(f"\n🔧 Testing: {params['name']} parameters")
        
        # Apply parameters
        controller.CROSS_MIN_ASPECT_RATIO = params['CROSS_MIN_ASPECT_RATIO']
        controller.CROSS_MIN_WIDTH_RATIO = params['CROSS_MIN_WIDTH_RATIO']
        controller.CROSS_MAX_HEIGHT_RATIO = params['CROSS_MAX_HEIGHT_RATIO']
        
        # Test detection
        detected, confidence, cross_center = controller.detect_camera_intersection()
        
        print(f"   Result: {detected} | {confidence}")
        if detected:
            print(f"   Center: {cross_center}")

if __name__ == "__main__":
    print("🧪 Testing Cross Detection Sensitivity")
    
    try:
        results = test_detection_sensitivity()
        print("\n" + "-" * 60 + "\n")
        test_parameter_tuning()
        
        print("\n✅ All tests completed!")
        print("💡 Check debug images for visual verification")
        
    except Exception as e:
        print(f"❌ Error during testing: {e}")
        import traceback
        traceback.print_exc()