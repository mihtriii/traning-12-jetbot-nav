#!/usr/bin/env python3
"""
Simple test cho black line detection
"""

import cv2
import numpy as np

def test_black_line_detection():
    """Test detection với line đen"""
    print("🧪 Testing Black Line Detection")
    print("=" * 40)
    
    # Tạo test image: line đen trên nền trắng
    width, height = 300, 300
    image = np.ones((height, width, 3), dtype=np.uint8) * 255  # Nền trắng
    
    # Main vertical line đen
    main_x = width // 2
    cv2.line(image, (main_x, 0), (main_x, height), (0, 0, 0), 8)
    
    # Cross horizontal line đen  
    cross_y = height // 2
    cv2.line(image, (0, cross_y), (width, cross_y), (0, 0, 0), 6)
    
    # Test với threshold method
    print("\n1. Testing grayscale threshold method:")
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    _, mask_thresh = cv2.threshold(gray, 60, 255, cv2.THRESH_BINARY_INV)
    
    # Count white pixels (detected line pixels)
    line_pixels = cv2.countNonZero(mask_thresh)
    total_pixels = width * height
    line_ratio = (line_pixels / total_pixels) * 100
    
    print(f"   Line pixels detected: {line_pixels}/{total_pixels} ({line_ratio:.1f}%)")
    
    # Test với HSV method (cũ)
    print("\n2. Testing HSV method:")
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    line_color_lower = np.array([0, 0, 0])
    line_color_upper = np.array([180, 255, 60])
    mask_hsv = cv2.inRange(hsv, line_color_lower, line_color_upper)
    
    line_pixels_hsv = cv2.countNonZero(mask_hsv)
    line_ratio_hsv = (line_pixels_hsv / total_pixels) * 100
    
    print(f"   Line pixels detected: {line_pixels_hsv}/{total_pixels} ({line_ratio_hsv:.1f}%)")
    
    # So sánh methods
    print(f"\n📊 Comparison:")
    print(f"   Threshold method: {line_ratio:.1f}% coverage")
    print(f"   HSV method: {line_ratio_hsv:.1f}% coverage")
    
    better_method = "Threshold" if line_pixels > line_pixels_hsv else "HSV"
    print(f"   🏆 Better method: {better_method}")
    
    # Save debug images
    cv2.imwrite("test_original.jpg", image)
    cv2.imwrite("test_threshold_mask.jpg", mask_thresh)
    cv2.imwrite("test_hsv_mask.jpg", mask_hsv)
    
    print(f"\n💾 Debug images saved:")
    print(f"   - test_original.jpg")
    print(f"   - test_threshold_mask.jpg") 
    print(f"   - test_hsv_mask.jpg")
    
    # Test contour detection
    print(f"\n3. Testing contour detection:")
    _, contours_thresh, _ = cv2.findContours(mask_thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    _, contours_hsv, _ = cv2.findContours(mask_hsv, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    print(f"   Threshold contours: {len(contours_thresh)}")
    print(f"   HSV contours: {len(contours_hsv)}")
    
    if len(contours_thresh) > 0:
        largest_contour = max(contours_thresh, key=cv2.contourArea)
        area = cv2.contourArea(largest_contour)
        x, y, w, h = cv2.boundingRect(largest_contour)
        aspect_ratio = w / h if h > 0 else 0
        
        print(f"   Largest contour area: {area}")
        print(f"   Bounding box: {w}x{h} (AR: {aspect_ratio:.2f})")
        
        # Check if it could be a cross line
        if aspect_ratio > 2.0 and w > width * 0.3:
            print(f"   ✅ Could be horizontal cross line!")
        else:
            print(f"   ❌ Not matching cross line criteria")
    
    return better_method

def test_different_thresholds():
    """Test với các threshold values khác nhau"""
    print("\n" + "=" * 40)
    print("🔧 Testing Different Threshold Values")
    print("=" * 40)
    
    # Tạo test image
    width, height = 300, 300
    image = np.ones((height, width, 3), dtype=np.uint8) * 255  # Nền trắng
    
    # Thêm line đen với độ đậm khác nhau
    cv2.line(image, (width//2, 0), (width//2, height), (0, 0, 0), 8)        # Đen hoàn toàn
    cv2.line(image, (0, height//2), (width, height//2), (30, 30, 30), 6)    # Xám đậm
    
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    
    threshold_values = [30, 60, 90, 120, 150]
    
    for thresh_val in threshold_values:
        _, mask = cv2.threshold(gray, thresh_val, 255, cv2.THRESH_BINARY_INV)
        line_pixels = cv2.countNonZero(mask)
        coverage = (line_pixels / (width * height)) * 100
        
        print(f"   Threshold {thresh_val:3d}: {line_pixels:5d} pixels ({coverage:4.1f}%)")
        
        # Save debug image
        cv2.imwrite(f"threshold_test_{thresh_val}.jpg", mask)
    
    print(f"\n💡 Recommended threshold: 60 (good balance)")
    print(f"💾 Debug images saved: threshold_test_*.jpg")

if __name__ == "__main__":
    print("🖤 Black Line Detection Test")
    
    better_method = test_black_line_detection()
    test_different_thresholds()
    
    print(f"\n✅ Test completed!")
    print(f"🎯 Recommendation: Use {better_method} method for black line detection")
    print(f"⚙️  Suggested threshold: 60 for THRESH_BINARY_INV")
    print(f"📝 Update LINE_COLOR_LOWER/UPPER if using HSV method")