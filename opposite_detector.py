#!/usr/bin/env python
# opposite_detector_node.py

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String, Bool
from std_srvs.srv import SetBool, SetBoolResponse
import time
import json
import math

class SimpleOppositeDetector:
    def __init__(self):
        # Các tham số cố định
        self.min_distance = 0.25
        self.max_distance = 0.4
        self.object_min_points = 10
        self.distance_threshold = 0.1
        self.angle_range = 10.0
        self.detection_interval = 2.0
        self.opposite_tolerance = 5.0
        self.min_opposite_distance = 45.0
        
        # Biến điều khiển
        self.scanning_active = False
        self.subscriber = None
        self.last_detection_time = 0
        self.latest_scan = None
        
        # Publisher để gửi tín hiệu giao lộ
#         self.notification_pub = rospy.Publisher('/intersection_trigger', String, queue_size=10)
#         self.status_pub = rospy.Publisher('/scan_status', Bool, queue_size=1)
        
        # Service để bật/tắt quét
#         self.start_service = rospy.Service('/start_scanning', SetBool, self.start_scanning_service)
#         self.stop_service = rospy.Service('/stop_scanning', SetBool, self.stop_scanning_service)
        
#         rospy.loginfo("Opposite Detector initialized.")
    
#     def start_scanning_service(self, req):
#         if req.data: return self.start_scanning()
#         else: return self.stop_scanning()
    
#     def stop_scanning_service(self, req):
#         if req.data: return self.stop_scanning()
#         else: return self.start_scanning()
    
    def start_scanning(self):
        try:
            if not self.scanning_active:
                self.subscriber = rospy.Subscriber('/scan', LaserScan, self.callback)
                self.scanning_active = True
#                 self.status_pub.publish(True)
#                 rospy.loginfo("Scanning STARTED")
                return SetBoolResponse(success=True, message="Scanning started")
            return SetBoolResponse(success=True, message="Already active")
        except Exception as e:
            rospy.logerr(f"Failed to start scanning: {e}")
            return SetBoolResponse(success=False, message=f"Failed to start: {e}")
    
    def stop_scanning(self):
        try:
            if self.scanning_active:
                if self.subscriber:
                    self.subscriber.unregister()
                    self.subscriber = None
                self.scanning_active = False
                self.latest_scan = None
#                 self.status_pub.publish(False)
                rospy.loginfo("Scanning STOPPED")
                return SetBoolResponse(success=True, message="Scanning stopped")
            return SetBoolResponse(success=True, message="Already inactive")
        except Exception as e:
            rospy.logerr(f"Failed to stop scanning: {e}")
            return SetBoolResponse(success=False, message=f"Failed to stop: {e}")

    def callback(self, scan):
        if not self.scanning_active: return
        self.latest_scan = scan
        current_time = time.time()
        if current_time - self.last_detection_time >= self.detection_interval:
            # self.process_detection()
            self.last_detection_time = current_time
    
    def print_all_lidar_data(self, scan):
        """In tất cả thông số LiDAR đo được"""
        print("\n" + "="*80)
        print("🔍 LIDAR SCAN DATA - COMPLETE ANALYSIS")
        print("="*80)
        
        # 1. Thông tin cơ bản về scan
        print(f"📊 SCAN BASIC INFO:")
        print(f"   • Timestamp: {scan.header.stamp}")
        print(f"   • Frame ID: {scan.header.frame_id}")
        print(f"   • Total ranges: {len(scan.ranges)}")
        print(f"   • Angle min: {math.degrees(scan.angle_min):.1f}°")
        print(f"   • Angle max: {math.degrees(scan.angle_max):.1f}°")
        print(f"   • Angle increment: {math.degrees(scan.angle_increment):.3f}°")
        print(f"   • Range min: {scan.range_min:.3f}m")
        print(f"   • Range max: {scan.range_max:.3f}m")
        print(f"   • Scan duration: {scan.scan_time:.3f}s")
        print(f"   • Time increment: {scan.time_increment:.6f}s")
        
        # 2. Phân tích dữ liệu ranges
        ranges = np.array(scan.ranges)
        valid_ranges = ranges[(ranges >= scan.range_min) & (ranges <= scan.range_max) & np.isfinite(ranges)]
        
        print(f"\n📏 RANGE STATISTICS:")
        print(f"   • Valid points: {len(valid_ranges)}/{len(ranges)} ({len(valid_ranges)/len(ranges)*100:.1f}%)")
        print(f"   • Invalid/Inf points: {len(ranges) - len(valid_ranges)}")
        
        if len(valid_ranges) > 0:
            print(f"   • Min distance: {np.min(valid_ranges):.3f}m")
            print(f"   • Max distance: {np.max(valid_ranges):.3f}m")
            print(f"   • Mean distance: {np.mean(valid_ranges):.3f}m")
            print(f"   • Median distance: {np.median(valid_ranges):.3f}m")
            print(f"   • Std deviation: {np.std(valid_ranges):.3f}m")
        
        # 3. Phân tích theo góc (8 hướng chính)
        print(f"\n🧭 DIRECTIONAL ANALYSIS:")
        directions = {
            'North (0°)': (-15, 15),
            'NE (45°)': (30, 60),
            'East (90°)': (75, 105),
            'SE (135°)': (120, 150),
            'South (180°)': (165, 195),
            'SW (225°)': (210, 240),
            'West (270°)': (255, 285),
            'NW (315°)': (300, 330)
        }
        
        for direction, (start_angle, end_angle) in directions.items():
            direction_ranges = []
            for i, distance in enumerate(ranges):
                angle_deg = math.degrees(scan.angle_min + i * scan.angle_increment)
                # Normalize angle to 0-360
                if angle_deg < 0:
                    angle_deg += 360
                
                if start_angle <= angle_deg <= end_angle and scan.range_min < distance < scan.range_max:
                    direction_ranges.append(distance)
            
            if direction_ranges:
                print(f"   • {direction}: {len(direction_ranges)} points, "
                      f"avg: {np.mean(direction_ranges):.3f}m, "
                      f"min: {np.min(direction_ranges):.3f}m")
            else:
                print(f"   • {direction}: No valid points")
        
        # 4. Phát hiện objects trong filter range
        print(f"\n🎯 OBJECTS IN DETECTION RANGE ({self.min_distance}m - {self.max_distance}m):")
        objects_in_range = []
        
        for i, distance in enumerate(ranges):
            if self.min_distance <= distance <= self.max_distance:
                angle_deg = math.degrees(scan.angle_min + i * scan.angle_increment)
                objects_in_range.append({
                    'index': i,
                    'angle': angle_deg,
                    'distance': distance
                })
        
        print(f"   • Total points in range: {len(objects_in_range)}")
        
        if objects_in_range:
            # Group by proximity
            clusters = []
            current_cluster = [objects_in_range[0]]
            
            for i in range(1, len(objects_in_range)):
                prev_point = objects_in_range[i-1]
                curr_point = objects_in_range[i]
                
                # Check if points are continuous (angle and distance)
                angle_diff = abs(curr_point['angle'] - prev_point['angle'])
                dist_diff = abs(curr_point['distance'] - prev_point['distance'])
                
                if angle_diff < 5.0 and dist_diff < self.distance_threshold:
                    current_cluster.append(curr_point)
                else:
                    if len(current_cluster) >= 3:  # Minimum points for object
                        clusters.append(current_cluster)
                    current_cluster = [curr_point]
            
            if len(current_cluster) >= 3:
                clusters.append(current_cluster)
            
            print(f"   • Detected clusters: {len(clusters)}")
            
            for j, cluster in enumerate(clusters):
                angles = [p['angle'] for p in cluster]
                distances = [p['distance'] for p in cluster]
                center_angle = np.mean(angles)
                avg_distance = np.mean(distances)
                
                print(f"     Cluster {j+1}: {len(cluster)} points, "
                      f"center: {center_angle:.1f}°, "
                      f"avg dist: {avg_distance:.3f}m, "
                      f"span: {np.min(angles):.1f}° to {np.max(angles):.1f}°")
        
        # 5. Closest and farthest valid points
        print(f"\n🔍 EXTREME POINTS:")
        if len(valid_ranges) > 0:
            min_idx = np.argmin(ranges[np.isfinite(ranges) & (ranges >= scan.range_min) & (ranges <= scan.range_max)])
            max_idx = np.argmax(ranges[np.isfinite(ranges) & (ranges >= scan.range_min) & (ranges <= scan.range_max)])
            
            min_angle = math.degrees(scan.angle_min + min_idx * scan.angle_increment)
            max_angle = math.degrees(scan.angle_min + max_idx * scan.angle_increment)
            
            print(f"   • Closest point: {np.min(valid_ranges):.3f}m at {min_angle:.1f}°")
            print(f"   • Farthest point: {np.max(valid_ranges):.3f}m at {max_angle:.1f}°")
        
        # 6. Points in front (important for navigation)
        print(f"\n⬆️  FRONT SECTOR ANALYSIS (-45° to +45°):")
        front_ranges = []
        front_angles = []
        
        for i, distance in enumerate(ranges):
            angle_deg = math.degrees(scan.angle_min + i * scan.angle_increment)
            if -45 <= angle_deg <= 45 and scan.range_min < distance < scan.range_max:
                front_ranges.append(distance)
                front_angles.append(angle_deg)
        
        if front_ranges:
            print(f"   • Valid front points: {len(front_ranges)}")
            print(f"   • Min front distance: {np.min(front_ranges):.3f}m")
            print(f"   • Avg front distance: {np.mean(front_ranges):.3f}m")
            
            # Find gaps in front (potential paths)
            far_points = [(angle, dist) for angle, dist in zip(front_angles, front_ranges) if dist > 1.0]
            if far_points:
                print(f"   • Open paths (>1.0m): {len(far_points)} points")
                for angle, dist in far_points[:5]:  # Show first 5
                    print(f"     - {angle:.1f}°: {dist:.3f}m")
        else:
            print(f"   • No valid points in front sector")
        
        print("="*80)
        print("🔍 END OF LIDAR ANALYSIS")
        print("="*80 + "\n")
    
    def index_to_angle(self, index, scan):
        angle_rad = scan.angle_min + (index * scan.angle_increment)
        return math.degrees(angle_rad)
    
    def get_angle_difference(self, angle1, angle2):
        diff = abs(angle1 - angle2)
        return 360 - diff if diff > 180 else diff
    
    def are_opposite(self, angle1, angle2):
        return abs(self.get_angle_difference(angle1, angle2) - 180.0) <= self.opposite_tolerance
    
    def find_all_objects(self, scan):
        # (logic phát hiện vật thể)
        ranges = np.array(scan.ranges)
        n = len(ranges)
        angle_increment_deg = math.degrees(scan.angle_increment)
        points_per_range = int(self.angle_range / angle_increment_deg)
        objects = []
        for start_idx in range(0, n, points_per_range // 2):
            end_idx = min(start_idx + points_per_range, n)
            if end_idx - start_idx < points_per_range // 2: continue
            zone_ranges = ranges[start_idx:end_idx]
            center_idx = start_idx + (end_idx - start_idx) // 2
            center_angle = self.index_to_angle(center_idx, scan)
            obj = self.detect_object_in_zone(zone_ranges, f"Zone_{start_idx}")
            if obj:
                obj['center_angle'] = center_angle
                obj['center_index'] = center_idx
                objects.append(obj)
        return objects

    def find_opposite_pairs(self, objects):
        opposite_pairs = []
        for i, obj1 in enumerate(objects):
            for j, obj2 in enumerate(objects[i+1:], i+1):
                angle_diff = self.get_angle_difference(obj1['center_angle'], obj2['center_angle'])
                if angle_diff >= self.min_opposite_distance and self.are_opposite(obj1['center_angle'], obj2['center_angle']):
                    opposite_pairs.append({'object1': obj1, 'object2': obj2, 'angle_difference': angle_diff})
        return opposite_pairs
    
    def process_detection(self):
        if self.latest_scan is None: return
        scan = self.latest_scan
        timestamp = rospy.get_time()
        all_objects = self.find_all_objects(scan)
        if len(all_objects) < 2: return
        
        opposite_pairs = self.find_opposite_pairs(all_objects)
        
        if opposite_pairs:
            opposite_pairs.sort(key=lambda x: abs(x['angle_difference'] - 180.0))
            best_pair = opposite_pairs[0]
            
            # 🔥 IN TẤT CẢ THÔNG SỐ LIDAR KHI PHÁT HIỆN OPPOSITE
            print(f"\n🚨 OPPOSITE OBJECTS DETECTED at {timestamp:.1f}s! 🚨")
            self.print_all_lidar_data(scan)
            
            # rospy.loginfo("[%.1f] *** OPPOSITE OBJECTS DETECTED ***", timestamp)
            # Tạo và gửi tin nhắn
            notification = {
                "timestamp": timestamp,
                "detection_type": 'OPPOSITE_OBJECTS',
                # ... thông tin chi tiết khác
            }
#             self.notification_pub.publish(json.dumps(notification))
            return True
        else:
            # rospy.loginfo("[%.1f] Found %d objects, but none are opposite", timestamp, len(all_objects))
            return False

    def detect_object_in_zone(self, zone_ranges, zone_name):
        if len(zone_ranges) == 0: return None
        valid_mask = (zone_ranges >= self.min_distance) & (zone_ranges <= self.max_distance) & np.isfinite(zone_ranges)
        if np.sum(valid_mask) < self.object_min_points: return None
        valid_ranges = zone_ranges[valid_mask]
        valid_indices = np.where(valid_mask)[0]
        clusters, current_cluster = [], [0]
        for i in range(1, len(valid_ranges)):
            if (valid_indices[i] - valid_indices[current_cluster[-1]] <= 2 and abs(valid_ranges[i] - valid_ranges[current_cluster[-1]]) <= self.distance_threshold):
                current_cluster.append(i)
            else:
                if len(current_cluster) >= self.object_min_points: clusters.append(current_cluster)
                current_cluster = [i]
        if len(current_cluster) >= self.object_min_points: clusters.append(current_cluster)
        if not clusters: return None
        largest_cluster = max(clusters, key=len)
        cluster_distances = [valid_ranges[i] for i in largest_cluster]
        return {'distance': np.mean(cluster_distances), 'point_count': len(largest_cluster), 'zone': zone_name}

# Test function để chạy và in lidar data
def test_lidar_data_printing():
    """Function để test việc in dữ liệu LiDAR"""
    rospy.init_node('lidar_data_printer', anonymous=True)
    
    detector = SimpleOppositeDetector()
    detector.start_scanning()
    
    print("🔍 Bắt đầu monitoring LiDAR data. Nhấn Ctrl+C để dừng.")
    print("📊 Sẽ in chi tiết khi phát hiện opposite objects...")
    
    rate = rospy.Rate(1)  # 1 Hz để không spam quá nhiều
    
    try:
        while not rospy.is_shutdown():
            if detector.latest_scan is not None:
                # Có thể uncomment dòng dưới để in mọi scan (cẩn thận - rất nhiều dữ liệu!)
                # detector.print_all_lidar_data(detector.latest_scan)
                
                # Hoặc chỉ process detection (sẽ in khi có opposite)
                detector.process_detection()
                
            rate.sleep()
            
    except rospy.ROSInterruptException:
        print("🛑 Dừng monitoring LiDAR data.")
    finally:
        detector.stop_scanning()

# if __name__ == '__main__':
#     test_lidar_data_printing()