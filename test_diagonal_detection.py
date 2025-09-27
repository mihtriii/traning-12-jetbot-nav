#!/usr/bin/env python3

import rospy
import numpy as np
import math
from sensor_msgs.msg import LaserScan
from opposite_detector import SimpleOppositeDetector

class DiagonalDetectorTest:
    def __init__(self):
        """Test diagonal detection with visualization."""
        rospy.init_node('diagonal_detector_test', anonymous=True)
        
        self.detector = SimpleOppositeDetector()
        self.detector.start_scanning()
        
        rospy.loginfo("🔧 Diagonal Detector Test initialized")
        rospy.loginfo("Only detecting objects near diagonal axes: 45°, 135°, 225°, 315°")
        
    def visualize_diagonal_zones(self):
        """Hiển thị các zone diagonal được monitor."""
        print("\n" + "="*60)
        print("🔍 DIAGONAL DETECTION ZONES")
        print("="*60)
        print("📐 Target diagonal angles:")
        print("   • 45° (front_right): 30° to 60°")
        print("   • 135° (back_right):  120° to 150°") 
        print("   • 225° (back_left):   210° to 240°")
        print("   • 315° (front_left):  300° to 330°")
        print("")
        print("🎯 Detection tolerance: ±15° around each diagonal")
        print("📏 Angle range per zone: 10°")
        print("📊 Distance range: 0.25m to 0.34m")
        print("="*60)
        
    def print_detection_results(self):
        """In kết quả phát hiện với thông tin chi tiết."""
        if self.detector.latest_scan is None:
            print("❌ No LiDAR data available")
            return
            
        scan = self.detector.latest_scan
        objects = self.detector.find_all_objects(scan)
        
        print(f"\n{'='*80}")
        print(f"🔍 DIAGONAL OBJECT DETECTION RESULTS")
        print(f"   Timestamp: {rospy.get_time():.1f}s")
        print(f"{'='*80}")
        
        if not objects:
            print("❌ No objects detected in diagonal zones")
            return
            
        print(f"✅ Found {len(objects)} objects in diagonal zones:")
        print("")
        
        # Nhóm objects theo diagonal type
        diagonal_groups = {}
        for obj in objects:
            diagonal_type = obj.get('diagonal_type', 'unknown')
            if diagonal_type not in diagonal_groups:
                diagonal_groups[diagonal_type] = []
            diagonal_groups[diagonal_type].append(obj)
        
        # In từng group
        for diagonal_type, group_objects in diagonal_groups.items():
            print(f"📐 {diagonal_type.upper()}:")
            for i, obj in enumerate(group_objects):
                angle = obj['center_angle']
                distance = obj['distance']
                points = obj['point_count']
                zone = obj['zone']
                
                print(f"   Object {i+1}:")
                print(f"     • Angle: {angle:6.1f}°")
                print(f"     • Distance: {distance:.3f}m")
                print(f"     • Points: {points}")
                print(f"     • Zone: {zone}")
                print("")
        
        # Kiểm tra opposite pairs
        opposite_pairs = self.detector.find_opposite_pairs(objects)
        
        if opposite_pairs:
            print(f"🎯 OPPOSITE PAIRS DETECTED: {len(opposite_pairs)}")
            for i, pair in enumerate(opposite_pairs):
                obj1 = pair['object1']
                obj2 = pair['object2'] 
                angle_diff = pair['angle_difference']
                
                print(f"   Pair {i+1}:")
                print(f"     • Object 1: {obj1['center_angle']:6.1f}° ({obj1.get('diagonal_type', 'unknown')})")
                print(f"     • Object 2: {obj2['center_angle']:6.1f}° ({obj2.get('diagonal_type', 'unknown')})")
                print(f"     • Angle difference: {angle_diff:.1f}°")
                print(f"     • Status: {'✅ INTERSECTION!' if abs(angle_diff - 180) <= 5 else '❌ Not opposite'}")
                print("")
        else:
            print("ℹ️  No opposite pairs detected")
            
        print("="*80)
    
    def run_continuous_test(self, interval=3.0):
        """Chạy test liên tục."""
        self.visualize_diagonal_zones()
        
        rospy.loginfo(f"🚀 Starting continuous diagonal detection test (every {interval}s)")
        rospy.loginfo("Press Ctrl+C to stop")
        
        rate = rospy.Rate(1/interval)
        
        try:
            while not rospy.is_shutdown():
                self.print_detection_results()
                rate.sleep()
                
        except rospy.ROSInterruptException:
            rospy.loginfo("🛑 Diagonal detection test stopped")
        finally:
            self.detector.stop_scanning()
    
    def create_mock_diagonal_data(self):
        """Tạo dữ liệu mock để test diagonal detection."""
        print("\n🔧 Creating mock data for diagonal detection test...")
        
        # Simulate objects at diagonal positions
        mock_objects = [
            {'center_angle': 45, 'distance': 0.30, 'point_count': 12, 'zone': 'Mock_45', 'diagonal_type': 'front_right'},
            {'center_angle': 135, 'distance': 0.28, 'point_count': 15, 'zone': 'Mock_135', 'diagonal_type': 'back_right'},
            {'center_angle': 225, 'distance': 0.32, 'point_count': 10, 'zone': 'Mock_225', 'diagonal_type': 'back_left'},
            {'center_angle': 315, 'distance': 0.29, 'point_count': 14, 'zone': 'Mock_315', 'diagonal_type': 'front_left'},
        ]
        
        print("✅ Mock diagonal objects created:")
        for obj in mock_objects:
            print(f"   • {obj['diagonal_type']}: {obj['center_angle']}° at {obj['distance']:.3f}m")
            
        # Test opposite pairs
        print("\n🎯 Testing opposite pair detection:")
        for i in range(len(mock_objects)):
            for j in range(i+1, len(mock_objects)):
                obj1 = mock_objects[i]
                obj2 = mock_objects[j]
                
                angle_diff = abs(obj1['center_angle'] - obj2['center_angle'])
                if angle_diff > 180:
                    angle_diff = 360 - angle_diff
                    
                is_opposite = abs(angle_diff - 180) <= 5
                
                print(f"   {obj1['diagonal_type']} vs {obj2['diagonal_type']}: {angle_diff:.0f}° {'✅' if is_opposite else '❌'}")
        
        return mock_objects


def main():
    """Main function"""
    import sys
    
    try:
        tester = DiagonalDetectorTest()
        
        if '--mock' in sys.argv:
            tester.create_mock_diagonal_data()
        elif '--single' in sys.argv:
            tester.visualize_diagonal_zones()
            rospy.sleep(2.0)  # Wait for data
            tester.print_detection_results()
        else:
            # Default: continuous mode
            interval = 3.0
            for arg in sys.argv:
                if arg.startswith('--interval='):
                    try:
                        interval = float(arg.split('=')[1])
                    except ValueError:
                        rospy.logwarn(f"Invalid interval, using default {interval}s")
            
            tester.run_continuous_test(interval)
            
    except rospy.ROSInterruptException:
        rospy.loginfo("Test interrupted")
    except Exception as e:
        rospy.logerr(f"Test error: {e}")


if __name__ == '__main__':
    main()