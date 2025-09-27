#!/usr/bin/env python3
"""
Test suite for anti-glare line detection improvements
Tests the improved algorithms against glare and lighting artifacts
"""

import cv2
import numpy as np
import time
import sys
import os

# Add current directory to path for imports
sys.path.append(os.path.dirname(__file__))

def create_glare_test_images():
    """Create test images with various glare and lighting conditions"""
    width, height = 300, 300
    test_cases = {}
    
    # Case 1: Perfect intersection (baseline)
    image1 = np.ones((height, width, 3), dtype=np.uint8) * 255
    cv2.line(image1, (width//2, 0), (width//2, height), (0, 0, 0), 8)
    cv2.line(image1, (0, height//2), (width, height//2), (0, 0, 0), 6)
    test_cases['perfect_baseline'] = image1
    
    # Case 2: Bright glare spots (simulate sunlight reflections)
    image2 = np.ones((height, width, 3), dtype=np.uint8) * 255
    cv2.line(image2, (width//2, 0), (width//2, height), (0, 0, 0), 8)
    cv2.line(image2, (0, height//2), (width, height//2), (0, 0, 0), 6)
    # Add bright glare spots
    for _ in range(8):
        x, y = np.random.randint(50, width-50), np.random.randint(50, height-50)
        cv2.circle(image2, (x, y), np.random.randint(10, 25), (255, 255, 255), -1)
    test_cases['bright_glare_spots'] = image2
    
    # Case 3: Uneven lighting (gradient from dark to bright)
    image3 = np.zeros((height, width, 3), dtype=np.uint8)
    for y in range(height):
        for x in range(width):
            # Create diagonal gradient
            intensity = int(100 + 150 * ((x + y) / (width + height)))
            image3[y, x] = [intensity, intensity, intensity]
    cv2.line(image3, (width//2, 0), (width//2, height), (0, 0, 0), 8)
    cv2.line(image3, (0, height//2), (width, height//2), (0, 0, 0), 6)
    test_cases['gradient_lighting'] = image3
    
    # Case 4: High contrast shadows and highlights
    image4 = np.ones((height, width, 3), dtype=np.uint8) * 200
    # Add dark shadow area
    cv2.rectangle(image4, (0, 0), (width//3, height), (80, 80, 80), -1)
    # Add bright highlight area
    cv2.rectangle(image4, (2*width//3, 0), (width, height), (255, 255, 255), -1)
    cv2.line(image4, (width//2, 0), (width//2, height), (0, 0, 0), 8)
    cv2.line(image4, (0, height//2), (width, height//2), (0, 0, 0), 6)
    test_cases['shadow_highlight'] = image4
    
    # Case 5: Random noise + glare
    image5 = np.ones((height, width, 3), dtype=np.uint8) * 200
    # Add random noise
    noise = np.random.normal(0, 40, (height, width, 3))
    image5 = np.clip(image5.astype(np.float32) + noise, 0, 255).astype(np.uint8)
    cv2.line(image5, (width//2, 0), (width//2, height), (0, 0, 0), 8)
    cv2.line(image5, (0, height//2), (width, height//2), (0, 0, 0), 6)
    # Add some glare artifacts
    for _ in range(5):
        x, y = np.random.randint(20, width-20), np.random.randint(20, height-20)
        w, h = np.random.randint(15, 40), np.random.randint(5, 15)
        cv2.rectangle(image5, (x, y), (x+w, y+h), (255, 255, 255), -1)
    test_cases['noisy_with_glare'] = image5
    
    # Case 6: Overexposed (very bright) image
    image6 = np.ones((height, width, 3), dtype=np.uint8) * 240
    cv2.line(image6, (width//2, 0), (width//2, height), (50, 50, 50), 8)  # Dark gray instead of black
    cv2.line(image6, (0, height//2), (width, height//2), (60, 60, 60), 6)
    test_cases['overexposed'] = image6
    
    # Case 7: Underexposed (very dark) image
    image7 = np.ones((height, width, 3), dtype=np.uint8) * 50
    cv2.line(image7, (width//2, 0), (width//2, height), (0, 0, 0), 8)
    cv2.line(image7, (0, height//2), (width, height//2), (0, 0, 0), 6)
    test_cases['underexposed'] = image7
    
    # Case 8: Reflection artifacts (irregular bright shapes)
    image8 = np.ones((height, width, 3), dtype=np.uint8) * 200
    cv2.line(image8, (width//2, 0), (width//2, height), (0, 0, 0), 8)
    cv2.line(image8, (0, height//2), (width, height//2), (0, 0, 0), 6)
    # Add irregular reflection shapes
    for _ in range(4):
        points = np.random.randint(20, width-20, (6, 2))
        cv2.fillPoly(image8, [points], (255, 255, 255))
    test_cases['reflection_artifacts'] = image8
    
    # Case 9: False positive test - bright horizontal artifacts (should NOT detect)
    image9 = np.ones((height, width, 3), dtype=np.uint8) * 200
    cv2.line(image9, (width//2, 0), (width//2, height), (0, 0, 0), 8)  # Only main line
    # Add bright horizontal artifacts that look like lines but aren't real
    for i in range(3):
        y = int(height * (0.3 + i * 0.15))
        # Irregular bright horizontal artifact
        for x in range(0, width, 10):
            w = np.random.randint(5, 20)
            cv2.rectangle(image9, (x, y-2), (x+w, y+2), (255, 255, 255), -1)
    test_cases['false_positive_artifacts'] = image9
    
    return test_cases

def test_preprocessing_methods(image, method_name):
    """Test different anti-glare preprocessing methods"""
    
    if method_name == "original":
        # No preprocessing
        return image
        
    elif method_name == "bilateral_only":
        # Only bilateral filter
        return cv2.bilateralFilter(image, 9, 75, 75)
        
    elif method_name == "histogram_eq_only":
        # Only histogram equalization
        yuv = cv2.cvtColor(image, cv2.COLOR_BGR2YUV)
        yuv[:,:,0] = cv2.equalizeHist(yuv[:,:,0])
        return cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR)
        
    elif method_name == "combined_anti_glare":
        # Full anti-glare pipeline
        filtered = cv2.bilateralFilter(image, 9, 75, 75)
        yuv = cv2.cvtColor(filtered, cv2.COLOR_BGR2YUV)
        yuv[:,:,0] = cv2.equalizeHist(yuv[:,:,0])
        return cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR)

def test_line_detection_with_antiglare(image, roi_y_percent=0.45, roi_h_percent=0.30):
    """Test line detection with anti-glare processing"""
    
    height, width = image.shape[:2]
    roi_y = int(height * roi_y_percent)
    roi_h = int(height * roi_h_percent)
    roi = image[roi_y:roi_y + roi_h, :]
    
    # Apply anti-glare preprocessing
    roi_filtered = cv2.bilateralFilter(roi, 9, 75, 75)
    roi_yuv = cv2.cvtColor(roi_filtered, cv2.COLOR_BGR2YUV)
    roi_yuv[:,:,0] = cv2.equalizeHist(roi_yuv[:,:,0])
    roi_processed = cv2.cvtColor(roi_yuv, cv2.COLOR_YUV2BGR)
    
    # Multi-method detection
    hsv = cv2.cvtColor(roi_processed, cv2.COLOR_BGR2HSV)
    hsv_mask = cv2.inRange(hsv, np.array([0, 0, 0]), np.array([180, 255, 120]))
    
    gray = cv2.cvtColor(roi_processed, cv2.COLOR_BGR2GRAY)
    gray_blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    _, thresh_mask = cv2.threshold(gray_blurred, 60, 255, cv2.THRESH_BINARY_INV)
    
    adaptive_mask = cv2.adaptiveThreshold(gray_blurred, 255, cv2.ADAPTIVE_THRESH_MEAN_C, 
                                        cv2.THRESH_BINARY_INV, 11, 4)
    
    combined_mask = cv2.bitwise_or(hsv_mask, thresh_mask)
    combined_mask = cv2.bitwise_or(combined_mask, adaptive_mask)
    combined_mask = cv2.medianBlur(combined_mask, 3)
    
    # Find contours with quality filtering
    _, contours, _ = cv2.findContours(combined_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    best_candidate = None
    best_score = 0
    
    if contours:
        roi_height, roi_width = combined_mask.shape
        
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < 50:
                continue
                
            x, y, w, h = cv2.boundingRect(contour)
            aspect_ratio = w / h if h > 0 else 0
            width_ratio = w / roi_width
            
            # Quality metrics for anti-glare
            perimeter = cv2.arcLength(contour, True)
            if perimeter == 0:
                continue
                
            hull = cv2.convexHull(contour)
            hull_area = cv2.contourArea(hull)
            solidity = area / hull_area if hull_area > 0 else 0
            extent = area / (w * h) if (w * h) > 0 else 0
            compactness = (4 * np.pi * area) / (perimeter * perimeter) if perimeter > 0 else 0
            
            # Enhanced filtering
            if (aspect_ratio > 1.5 and width_ratio > 0.3 and
                solidity > 0.2 and solidity < 0.95 and
                extent > 0.3 and compactness > 0.01):
                
                # Scoring
                area_score = min(area / (roi_width * roi_height), 1.0)
                center_score = 1.0 - abs((x + w/2) - roi_width/2) / (roi_width/2)
                quality_score = (solidity + extent + min(compactness * 10, 1.0)) / 3.0
                
                total_score = area_score * 0.4 + center_score * 0.3 + quality_score * 0.3
                
                if total_score > best_score:
                    best_score = total_score
                    best_candidate = {
                        'area': area,
                        'aspect_ratio': aspect_ratio,
                        'width_ratio': width_ratio,
                        'solidity': solidity,
                        'extent': extent,
                        'compactness': compactness,
                        'score': total_score,
                        'bbox': (x, y, w, h)
                    }
    
    detected = best_candidate is not None
    confidence = "HIGH" if best_score > 0.65 else "MEDIUM" if best_score > 0.40 else "LOW"
    
    return detected, confidence, best_candidate, combined_mask

def run_anti_glare_test():
    """Run comprehensive anti-glare test"""
    print("🛡️  ANTI-GLARE LINE DETECTION TEST")
    print("=" * 80)
    
    test_images = create_glare_test_images()
    
    results = {}
    total_tests = 0
    successful_detections = 0
    false_positives = 0
    
    for case_name, image in test_images.items():
        print(f"\n🧪 Testing: {case_name.replace('_', ' ').title()}")
        
        # Save original test image
        cv2.imwrite(f"antiglare_test_{case_name}.jpg", image)
        
        # Test detection
        detected, confidence, candidate, mask = test_line_detection_with_antiglare(image)
        
        results[case_name] = {
            'detected': detected,
            'confidence': confidence,
            'candidate': candidate
        }
        
        # Determine if this should be detected
        should_detect = not case_name.startswith('false_positive')
        
        if should_detect:
            total_tests += 1
            if detected:
                successful_detections += 1
                status = f"✅ DETECTED ({confidence})"
                if candidate:
                    status += f" - Quality: S={candidate['solidity']:.2f}, E={candidate['extent']:.2f}, C={candidate['compactness']:.3f}"
            else:
                status = f"❌ MISSED ({confidence})"
        else:
            # False positive test
            if detected:
                false_positives += 1
                status = f"⚠️  FALSE POSITIVE ({confidence})"
            else:
                status = f"✅ CORRECTLY REJECTED ({confidence})"
        
        print(f"  Result: {status}")
        
        # Save debug mask
        cv2.imwrite(f"antiglare_mask_{case_name}.jpg", mask)
    
    # Calculate performance metrics
    detection_rate = (successful_detections / total_tests) * 100 if total_tests > 0 else 0
    
    print("\n" + "=" * 80)
    print("📊 ANTI-GLARE PERFORMANCE SUMMARY")
    print("=" * 80)
    print(f"Detection Rate: {successful_detections}/{total_tests} ({detection_rate:.1f}%)")
    print(f"False Positives: {false_positives}")
    print(f"Overall Performance: {'EXCELLENT' if detection_rate > 80 and false_positives == 0 else 'GOOD' if detection_rate > 70 else 'NEEDS IMPROVEMENT'}")
    
    # Detailed analysis
    print(f"\n📋 DETAILED RESULTS:")
    for case_name, result in results.items():
        should_detect = not case_name.startswith('false_positive')
        expected = "DETECT" if should_detect else "REJECT"
        actual = "DETECT" if result['detected'] else "REJECT"
        status = "✅" if (should_detect == result['detected']) else "❌"
        
        print(f"  {status} {case_name:<25} | Expected: {expected:6} | Actual: {actual:6} | Conf: {result['confidence']}")
    
    return results

if __name__ == "__main__":
    print("🛡️  Anti-Glare Line Detection Test Suite")
    print("Testing improved algorithms against lighting artifacts and glare")
    
    try:
        results = run_anti_glare_test()
        
        print(f"\n✅ Anti-glare test completed!")
        print(f"📁 Check generated test images: antiglare_test_*.jpg")
        print(f"🔍 Check debug masks: antiglare_mask_*.jpg")
        print(f"\n💡 IMPROVEMENTS APPLIED:")
        print(f"  • Bilateral filtering for noise reduction")
        print(f"  • Histogram equalization for lighting normalization") 
        print(f"  • Multi-method detection (HSV + Grayscale + Adaptive)")
        print(f"  • Contour quality filtering (solidity, extent, compactness)")
        print(f"  • Enhanced scoring with quality metrics")
        
    except Exception as e:
        print(f"❌ Test failed: {e}")
        import traceback
        traceback.print_exc()