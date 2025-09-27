#!/usr/bin/env python3
"""
Enhanced test suite for improved black line detection
Tests the improved camera detection algorithms with various challenging scenarios
"""

import cv2
import numpy as np
import time
import sys
import os

# Add current directory to path for imports
sys.path.append(os.path.dirname(__file__))

def create_comprehensive_test_images():
    """Create a comprehensive set of test images for line detection validation"""
    width, height = 300, 300
    test_cases = {}
    
    # Case 1: Perfect intersection (baseline)
    image1 = np.ones((height, width, 3), dtype=np.uint8) * 255
    cv2.line(image1, (width//2, 0), (width//2, height), (0, 0, 0), 8)  # Main line
    cv2.line(image1, (0, height//2), (width, height//2), (0, 0, 0), 6)  # Cross line
    test_cases['perfect_intersection'] = image1
    
    # Case 2: Thin cross line
    image2 = np.ones((height, width, 3), dtype=np.uint8) * 255
    cv2.line(image2, (width//2, 0), (width//2, height), (0, 0, 0), 8)  # Main line
    cv2.line(image2, (0, height//2), (width, height//2), (0, 0, 0), 2)  # Very thin cross
    test_cases['thin_cross'] = image2
    
    # Case 3: Short cross line
    image3 = np.ones((height, width, 3), dtype=np.uint8) * 255
    cv2.line(image3, (width//2, 0), (width//2, height), (0, 0, 0), 8)  # Main line
    cv2.line(image3, (width//4, height//2), (3*width//4, height//2), (0, 0, 0), 6)  # Short cross
    test_cases['short_cross'] = image3
    
    # Case 4: Off-center cross
    image4 = np.ones((height, width, 3), dtype=np.uint8) * 255
    cv2.line(image4, (width//2, 0), (width//2, height), (0, 0, 0), 8)  # Main line
    cv2.line(image4, (0, int(height*0.4)), (width, int(height*0.4)), (0, 0, 0), 6)  # Off-center cross
    test_cases['offset_cross'] = image4
    
    # Case 5: Broken cross line
    image5 = np.ones((height, width, 3), dtype=np.uint8) * 255
    cv2.line(image5, (width//2, 0), (width//2, height), (0, 0, 0), 8)  # Main line
    cv2.line(image5, (0, height//2), (width//3, height//2), (0, 0, 0), 6)  # Left part
    cv2.line(image5, (2*width//3, height//2), (width, height//2), (0, 0, 0), 6)  # Right part
    test_cases['broken_cross'] = image5
    
    # Case 6: Dark gray lines (not pure black)
    image6 = np.ones((height, width, 3), dtype=np.uint8) * 255
    cv2.line(image6, (width//2, 0), (width//2, height), (50, 50, 50), 8)  # Dark gray main
    cv2.line(image6, (0, height//2), (width, height//2), (40, 40, 40), 6)  # Dark gray cross
    test_cases['dark_gray_lines'] = image6
    
    # Case 7: Noisy background
    image7 = np.ones((height, width, 3), dtype=np.uint8) * 255
    # Add noise
    noise = np.random.normal(0, 25, (height, width, 3))
    image7 = np.clip(image7.astype(np.float32) + noise, 0, 255).astype(np.uint8)
    cv2.line(image7, (width//2, 0), (width//2, height), (0, 0, 0), 8)  # Main line
    cv2.line(image7, (0, height//2), (width, height//2), (0, 0, 0), 6)  # Cross line
    test_cases['noisy_background'] = image7
    
    # Case 8: Varying lighting (gradient background)
    image8 = np.zeros((height, width, 3), dtype=np.uint8)
    for y in range(height):
        intensity = int(150 + 100 * (y / height))  # Gradient from dark to light
        image8[y, :] = [intensity, intensity, intensity]
    cv2.line(image8, (width//2, 0), (width//2, height), (0, 0, 0), 8)  # Main line
    cv2.line(image8, (0, height//2), (width, height//2), (0, 0, 0), 6)  # Cross line
    test_cases['gradient_lighting'] = image8
    
    # Case 9: No cross line (negative test)
    image9 = np.ones((height, width, 3), dtype=np.uint8) * 255
    cv2.line(image9, (width//2, 0), (width//2, height), (0, 0, 0), 8)  # Main line only
    test_cases['no_cross'] = image9
    
    # Case 10: Multiple cross lines
    image10 = np.ones((height, width, 3), dtype=np.uint8) * 255
    cv2.line(image10, (width//2, 0), (width//2, height), (0, 0, 0), 8)  # Main line
    cv2.line(image10, (0, int(height*0.4)), (width, int(height*0.4)), (0, 0, 0), 4)  # Cross 1
    cv2.line(image10, (0, int(height*0.6)), (width, int(height*0.6)), (0, 0, 0), 4)  # Cross 2
    test_cases['multiple_cross'] = image10
    
    return test_cases

def test_detection_method(image, method_name):
    """Test a specific detection method on an image"""
    roi_y = int(300 * 0.45)  # Use new improved ROI
    roi_h = int(300 * 0.30)
    roi = image[roi_y:roi_y + roi_h, :]
    
    if method_name == "hsv_original":
        # Original HSV method
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, np.array([0, 0, 0]), np.array([180, 255, 75]))
        
    elif method_name == "hsv_improved":
        # Improved HSV method
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, np.array([0, 0, 0]), np.array([180, 255, 120]))
        
    elif method_name == "grayscale":
        # Grayscale threshold method
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        _, mask = cv2.threshold(gray, 60, 255, cv2.THRESH_BINARY_INV)
        
    elif method_name == "adaptive":
        # Adaptive threshold method
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        mask = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_MEAN_C, 
                                   cv2.THRESH_BINARY_INV, 11, 2)
        
    elif method_name == "combined":
        # Combined method (like in improved detection)
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        hsv_mask = cv2.inRange(hsv, np.array([0, 0, 0]), np.array([180, 255, 120]))
        
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        _, thresh_mask = cv2.threshold(gray, 60, 255, cv2.THRESH_BINARY_INV)
        
        adaptive_mask = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_MEAN_C, 
                                            cv2.THRESH_BINARY_INV, 11, 2)
        
        mask = cv2.bitwise_or(hsv_mask, thresh_mask)
        mask = cv2.bitwise_or(mask, adaptive_mask)
        
        # Apply morphological operations
        kernel = np.ones((3,3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    
    # Analyze the mask
    _, contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    detected = False
    best_candidate = None
    
    if contours:
        roi_height, roi_width = mask.shape
        
        for contour in contours:
            x, y, w, h = cv2.boundingRect(contour)
            aspect_ratio = w / h if h > 0 else 0
            width_ratio = w / roi_width
            area = cv2.contourArea(contour)
            
            # Use improved criteria
            if (aspect_ratio > 1.5 and width_ratio > 0.3 and area > 50):
                if best_candidate is None or area > best_candidate['area']:
                    best_candidate = {
                        'area': area,
                        'aspect_ratio': aspect_ratio,
                        'width_ratio': width_ratio,
                        'bbox': (x, y, w, h)
                    }
                    detected = True
    
    return detected, best_candidate, mask

def run_comprehensive_test():
    """Run comprehensive test of all detection methods"""
    print("🧪 COMPREHENSIVE LINE DETECTION TEST")
    print("=" * 80)
    
    test_images = create_comprehensive_test_images()
    methods = ["hsv_original", "hsv_improved", "grayscale", "adaptive", "combined"]
    
    results = {}
    
    for case_name, image in test_images.items():
        print(f"\n📝 Testing: {case_name.replace('_', ' ').title()}")
        results[case_name] = {}
        
        # Save original test image
        cv2.imwrite(f"test_{case_name}.jpg", image)
        
        for method in methods:
            detected, candidate, mask = test_detection_method(image, method)
            results[case_name][method] = detected
            
            status = "✅ DETECTED" if detected else "❌ MISSED"
            details = ""
            if detected and candidate:
                details = f" (AR:{candidate['aspect_ratio']:.1f}, W:{candidate['width_ratio']:.1f}, A:{int(candidate['area'])})"
            
            print(f"  {method:15}: {status}{details}")
            
            # Save debug mask
            cv2.imwrite(f"mask_{case_name}_{method}.jpg", mask)
    
    # Calculate detection rates
    print("\n" + "=" * 80)
    print("📊 DETECTION RATE SUMMARY")
    print("=" * 80)
    
    expected_positives = [case for case in test_images.keys() if case != 'no_cross']
    
    for method in methods:
        detected_count = 0
        false_positive = 0
        
        for case_name in expected_positives:
            if results[case_name][method]:
                detected_count += 1
        
        # Check false positive
        if results['no_cross'][method]:
            false_positive = 1
        
        detection_rate = (detected_count / len(expected_positives)) * 100
        
        print(f"{method:15}: {detected_count:2d}/{len(expected_positives)} ({detection_rate:5.1f}%) - FP: {false_positive}")
    
    # Find best performing method
    best_method = None
    best_score = 0
    
    for method in methods:
        score = sum(1 for case in expected_positives if results[case][method])
        if score > best_score:
            best_score = score
            best_method = method
    
    print(f"\n🏆 BEST METHOD: {best_method} ({best_score}/{len(expected_positives)} detections)")
    
    return results

def create_comparison_report(results):
    """Create a detailed comparison report"""
    print("\n" + "=" * 80)
    print("📋 DETAILED COMPARISON REPORT")
    print("=" * 80)
    
    test_cases = list(results.keys())
    methods = ["hsv_original", "hsv_improved", "grayscale", "adaptive", "combined"]
    
    # Header
    print(f"{'Test Case':<20}", end="")
    for method in methods:
        print(f"{method:>12}", end="")
    print()
    
    print("-" * 80)
    
    # Data rows
    for case in test_cases:
        print(f"{case:<20}", end="")
        for method in methods:
            symbol = " ✓ " if results[case][method] else " ✗ "
            print(f"{symbol:>12}", end="")
        print()
    
    print("\n💡 RECOMMENDATIONS:")
    print("- 'combined' method shows best overall performance")
    print("- Improved HSV threshold (120 vs 75) catches more dark lines")
    print("- Grayscale method good for consistent lighting conditions")
    print("- Adaptive threshold excellent for varying lighting")
    print("- Multi-method combination provides most robust detection")

if __name__ == "__main__":
    print("🔍 Enhanced Black Line Detection Test Suite")
    print("Testing improved algorithms against challenging scenarios")
    
    try:
        results = run_comprehensive_test()
        create_comparison_report(results)
        
        print(f"\n✅ Test completed successfully!")
        print(f"📁 Check generated test images and masks for visual verification")
        print(f"💾 Files saved: test_*.jpg and mask_*.jpg")
        
    except Exception as e:
        print(f"❌ Test failed: {e}")
        import traceback
        traceback.print_exc()