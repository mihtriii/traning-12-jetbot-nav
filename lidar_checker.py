#!/usr/bin/env python3

import rospy
import numpy as np
import math
import time
from sensor_msgs.msg import LaserScan

class LidarChecker:
    def __init__(self):
        """
        Khởi tạo LiDAR checker để kiểm tra và phân tích dữ liệu LiDAR.
        """
        rospy.loginfo("🔍 LiDAR Checker initialized")
        
        # LiDAR parameters
        self.latest_scan = None
        self.scan_count = 0
        self.start_time = time.time()
        
        # Detection parameters
        self.min_distance = 0.15
        self.max_distance = 2.0
        self.front_angle_range = 45  # ±45 degrees for front sector
        
        # ROS subscriber
        rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        
        rospy.loginfo("Waiting for LiDAR data on /scan topic...")
    
    def scan_callback(self, scan):
        """Callback nhận dữ liệu LiDAR."""
        self.latest_scan = scan
        self.scan_count += 1
        
        if self.scan_count % 10 == 0:  # Log every 10 scans
            rospy.loginfo(f"📡 Received {self.scan_count} scans")
    
    def print_basic_info(self, scan):
        """In thông tin cơ bản về scan."""
        print(f"\n{'='*60}")
        print(f"📊 LIDAR BASIC INFO")
        print(f"{'='*60}")
        print(f"   • Timestamp: {scan.header.stamp}")
        print(f"   • Frame ID: {scan.header.frame_id}")
        print(f"   • Total points: {len(scan.ranges)}")
        print(f"   • Angle range: {math.degrees(scan.angle_min):.1f}° to {math.degrees(scan.angle_max):.1f}°")
        print(f"   • Angle increment: {math.degrees(scan.angle_increment):.3f}°")
        print(f"   • Distance range: {scan.range_min:.3f}m to {scan.range_max:.3f}m")
        print(f"   • Scan duration: {scan.scan_time:.3f}s")
    
    def analyze_ranges(self, scan):
        """Phân tích dữ liệu khoảng cách."""
        ranges = np.array(scan.ranges)
        
        # Filter valid ranges
        valid_mask = (ranges >= scan.range_min) & (ranges <= scan.range_max) & np.isfinite(ranges)
        valid_ranges = ranges[valid_mask]
        invalid_count = len(ranges) - len(valid_ranges)
        
        print(f"\n📏 RANGE ANALYSIS:")
        print(f"   • Valid points: {len(valid_ranges)}/{len(ranges)} ({len(valid_ranges)/len(ranges)*100:.1f}%)")
        print(f"   • Invalid/Inf points: {invalid_count}")
        
        if len(valid_ranges) > 0:
            print(f"   • Min distance: {np.min(valid_ranges):.3f}m")
            print(f"   • Max distance: {np.max(valid_ranges):.3f}m")
            print(f"   • Mean distance: {np.mean(valid_ranges):.3f}m")
            print(f"   • Median distance: {np.median(valid_ranges):.3f}m")
            print(f"   • Std deviation: {np.std(valid_ranges):.3f}m")
    
    def analyze_front_sector(self, scan):
        """Phân tích khu vực phía trước (quan trọng cho navigation)."""
        ranges = np.array(scan.ranges)
        front_ranges = []
        front_angles = []
        
        for i, distance in enumerate(ranges):
            angle_deg = math.degrees(scan.angle_min + i * scan.angle_increment)
            
            # Normalize angle to -180 to +180
            if angle_deg > 180:
                angle_deg -= 360
            elif angle_deg < -180:
                angle_deg += 360
            
            # Check if in front sector
            if -self.front_angle_range <= angle_deg <= self.front_angle_range:
                if scan.range_min < distance < scan.range_max:
                    front_ranges.append(distance)
                    front_angles.append(angle_deg)
        
        print(f"\n⬆️  FRONT SECTOR ANALYSIS (±{self.front_angle_range}°):")
        
        if front_ranges:
            print(f"   • Valid front points: {len(front_ranges)}")
            print(f"   • Min front distance: {np.min(front_ranges):.3f}m")
            print(f"   • Max front distance: {np.max(front_ranges):.3f}m")
            print(f"   • Avg front distance: {np.mean(front_ranges):.3f}m")
            
            # Find closest obstacle
            min_idx = np.argmin(front_ranges)
            closest_angle = front_angles[min_idx]
            closest_dist = front_ranges[min_idx]
            
            print(f"   • Closest obstacle: {closest_dist:.3f}m at {closest_angle:.1f}°")
            
            # Categorize distances
            very_close = [d for d in front_ranges if d < 0.3]
            close = [d for d in front_ranges if 0.3 <= d < 0.8]
            medium = [d for d in front_ranges if 0.8 <= d < 1.5]
            far = [d for d in front_ranges if d >= 1.5]
            
            print(f"   • Very close (<0.3m): {len(very_close)} points")
            print(f"   • Close (0.3-0.8m): {len(close)} points")
            print(f"   • Medium (0.8-1.5m): {len(medium)} points")
            print(f"   • Far (>1.5m): {len(far)} points")
            
            # Navigation advice
            if len(very_close) > 5:
                print(f"   • ⚠️  WARNING: Multiple very close obstacles detected!")
            elif len(close) > 10:
                print(f"   • ⚠️  CAUTION: Close obstacles ahead")
            elif len(far) > len(front_ranges) * 0.7:
                print(f"   • ✅ Path ahead looks clear")
        else:
            print(f"   • ❌ No valid points in front sector")
    
    def find_objects_in_detection_range(self, scan):
        """Tìm các object trong khoảng detection range."""
        ranges = np.array(scan.ranges)
        objects_in_range = []
        
        for i, distance in enumerate(ranges):
            if self.min_distance <= distance <= self.max_distance:
                angle_deg = math.degrees(scan.angle_min + i * scan.angle_increment)
                
                # Normalize angle
                if angle_deg > 180:
                    angle_deg -= 360
                elif angle_deg < -180:
                    angle_deg += 360
                
                objects_in_range.append({
                    'index': i,
                    'angle': angle_deg,
                    'distance': distance
                })
        
        print(f"\n🎯 OBJECTS IN DETECTION RANGE ({self.min_distance}m - {self.max_distance}m):")
        print(f"   • Total points in range: {len(objects_in_range)}")
        
        if objects_in_range:
            # Group nearby points into clusters
            clusters = self.group_into_clusters(objects_in_range)
            print(f"   • Detected clusters: {len(clusters)}")
            
            for j, cluster in enumerate(clusters):
                angles = [p['angle'] for p in cluster]
                distances = [p['distance'] for p in cluster]
                center_angle = np.mean(angles)
                avg_distance = np.mean(distances)
                angle_span = np.max(angles) - np.min(angles)
                
                print(f"     Cluster {j+1}: {len(cluster)} points")
                print(f"       • Center: {center_angle:.1f}°")
                print(f"       • Avg distance: {avg_distance:.3f}m")
                print(f"       • Angle span: {angle_span:.1f}°")
                print(f"       • Range: {center_angle-angle_span/2:.1f}° to {center_angle+angle_span/2:.1f}°")
    
    def group_into_clusters(self, points):
        """Nhóm các điểm gần nhau thành clusters."""
        if not points:
            return []
        
        # Sort by angle
        points.sort(key=lambda p: p['angle'])
        
        clusters = []
        current_cluster = [points[0]]
        
        for i in range(1, len(points)):
            prev_point = points[i-1]
            curr_point = points[i]
            
            # Check if points should be in same cluster
            angle_diff = abs(curr_point['angle'] - prev_point['angle'])
            dist_diff = abs(curr_point['distance'] - prev_point['distance'])
            
            # Points are in same cluster if they're close in angle and distance
            if angle_diff < 5.0 and dist_diff < 0.1:
                current_cluster.append(curr_point)
            else:
                # Start new cluster
                if len(current_cluster) >= 3:  # Minimum points for valid cluster
                    clusters.append(current_cluster)
                current_cluster = [curr_point]
        
        # Don't forget the last cluster
        if len(current_cluster) >= 3:
            clusters.append(current_cluster)
        
        return clusters
    
    def analyze_scan_quality(self, scan):
        """Phân tích chất lượng của scan."""
        ranges = np.array(scan.ranges)
        
        # Count different types of readings
        valid_count = np.sum((ranges >= scan.range_min) & (ranges <= scan.range_max) & np.isfinite(ranges))
        inf_count = np.sum(np.isinf(ranges))
        nan_count = np.sum(np.isnan(ranges))
        too_close_count = np.sum(ranges < scan.range_min)
        too_far_count = np.sum(ranges > scan.range_max)
        
        print(f"\n📈 SCAN QUALITY ANALYSIS:")
        print(f"   • Valid readings: {valid_count}/{len(ranges)} ({valid_count/len(ranges)*100:.1f}%)")
        print(f"   • Infinite readings: {inf_count} ({inf_count/len(ranges)*100:.1f}%)")
        print(f"   • NaN readings: {nan_count} ({nan_count/len(ranges)*100:.1f}%)")
        print(f"   • Too close readings: {too_close_count}")
        print(f"   • Too far readings: {too_far_count}")
        
        # Quality rating
        quality_score = valid_count / len(ranges) * 100
        
        if quality_score > 90:
            quality = "EXCELLENT"
        elif quality_score > 80:
            quality = "GOOD"
        elif quality_score > 70:
            quality = "FAIR"
        elif quality_score > 50:
            quality = "POOR"
        else:
            quality = "VERY POOR"
        
        print(f"   • Overall quality: {quality} ({quality_score:.1f}%)")
    
    def print_full_analysis(self):
        """In phân tích đầy đủ của scan hiện tại."""
        if self.latest_scan is None:
            print("❌ No LiDAR data available")
            return
        
        scan = self.latest_scan
        current_time = time.time()
        
        print(f"\n{'='*80}")
        print(f"🔍 COMPLETE LIDAR ANALYSIS")
        print(f"   Time: {current_time - self.start_time:.1f}s since start")
        print(f"   Scan count: {self.scan_count}")
        print(f"{'='*80}")
        
        self.print_basic_info(scan)
        self.analyze_ranges(scan)
        self.analyze_front_sector(scan)
        self.find_objects_in_detection_range(scan)
        self.analyze_scan_quality(scan)
        
        print(f"{'='*80}")
        print(f"🔍 END OF ANALYSIS")
        print(f"{'='*80}\n")
    
    def run_continuous_check(self, interval=2.0):
        """Chạy kiểm tra liên tục với interval (seconds)."""
        rospy.loginfo(f"🚀 Starting continuous LiDAR monitoring (every {interval}s)")
        rospy.loginfo("Press Ctrl+C to stop")
        
        rate = rospy.Rate(1/interval)  # Convert interval to frequency
        
        try:
            while not rospy.is_shutdown():
                if self.latest_scan is not None:
                    self.print_full_analysis()
                else:
                    elapsed = time.time() - self.start_time
                    print(f"⏳ Waiting for LiDAR data... ({elapsed:.1f}s)")
                
                rate.sleep()
                
        except rospy.ROSInterruptException:
            rospy.loginfo("🛑 LiDAR monitoring stopped")
    
    def run_single_check(self):
        """Chạy một lần kiểm tra và thoát."""
        rospy.loginfo("🔍 Waiting for one LiDAR scan...")
        
        timeout = 10.0  # 10 seconds timeout
        start_time = time.time()
        
        while self.latest_scan is None and not rospy.is_shutdown():
            if time.time() - start_time > timeout:
                rospy.logerr(f"❌ Timeout: No LiDAR data received after {timeout}s")
                return False
                
            rospy.sleep(0.1)
        
        if self.latest_scan is not None:
            self.print_full_analysis()
            return True
        
        return False


def main():
    """Main function"""
    rospy.init_node('lidar_checker_node', anonymous=True)
    
    # Check command line arguments
    import sys
    
    continuous_mode = '--continuous' in sys.argv or '-c' in sys.argv
    interval = 2.0
    
    # Check for custom interval
    for arg in sys.argv:
        if arg.startswith('--interval='):
            try:
                interval = float(arg.split('=')[1])
            except ValueError:
                rospy.logwarn(f"Invalid interval value, using default {interval}s")
    
    try:
        checker = LidarChecker()
        
        if continuous_mode:
            rospy.loginfo("🔄 Running in continuous mode")
            checker.run_continuous_check(interval)
        else:
            rospy.loginfo("🔍 Running single check mode")
            success = checker.run_single_check()
            
            if success:
                rospy.loginfo("✅ LiDAR check completed successfully")
            else:
                rospy.logerr("❌ LiDAR check failed")
                
    except rospy.ROSInterruptException:
        rospy.loginfo("Node interrupted")
    except Exception as e:
        rospy.logerr(f"Error: {e}")
        import traceback
        rospy.logerr(f"Traceback: {traceback.format_exc()}")


if __name__ == '__main__':
    main()