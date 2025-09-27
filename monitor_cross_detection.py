#!/usr/bin/env python3
"""
Real-time monitor script cho cross detection
Hiển thị detection performance và debug info
"""

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import time

class CrossDetectionMonitor:
    def __init__(self):
        rospy.init_node('cross_detection_monitor')
        
        # Import main controller
        from ros_lidar_follower import JetBotController
        self.controller = JetBotController()
        
        self.bridge = CvBridge()
        
        # Statistics
        self.total_frames = 0
        self.detection_count = 0
        self.last_detection_time = 0
        self.detection_history = []
        
        # Subscribe to camera
        rospy.Subscriber('/csi_cam_0/image_raw', Image, self.image_callback)
        
        rospy.loginfo("🔍 Cross Detection Monitor started!")
        rospy.loginfo("Press 'q' to quit, 's' to save current frame")
        
    def image_callback(self, msg):
        """Process incoming image and run detection"""
        try:
            # Convert ROS image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.controller.latest_image = cv2.resize(cv_image, (self.controller.WIDTH, self.controller.HEIGHT))
            
            self.process_frame()
            
        except Exception as e:
            rospy.logerr(f"Error processing image: {e}")
    
    def process_frame(self):
        """Process current frame and display results"""
        if self.controller.latest_image is None:
            return
        
        self.total_frames += 1
        current_time = rospy.get_time()
        
        # Run detection
        detected, confidence, cross_center = self.controller.detect_camera_intersection()
        
        # Update statistics
        if detected:
            self.detection_count += 1
            self.last_detection_time = current_time
        
        # Add to history
        self.detection_history.append({
            'timestamp': current_time,
            'detected': detected,
            'confidence': confidence,
            'center': cross_center
        })
        
        # Keep only last 100 records
        if len(self.detection_history) > 100:
            self.detection_history.pop(0)
        
        # Create display image
        display_image = self.create_display_image(detected, confidence, cross_center)
        
        # Show image
        cv2.imshow('Cross Detection Monitor', display_image)
        
        # Log every 30 frames
        if self.total_frames % 30 == 0:
            detection_rate = (self.detection_count / self.total_frames) * 100
            recent_detections = sum(1 for d in self.detection_history[-10:] if d['detected'])
            rospy.loginfo(f"📊 Frames: {self.total_frames}, "
                         f"Detection Rate: {detection_rate:.1f}%, "
                         f"Recent (last 10): {recent_detections}/10")
        
        # Handle key press
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            rospy.signal_shutdown('User requested quit')
        elif key == ord('s'):
            filename = f"cross_detection_frame_{int(current_time)}.jpg"
            cv2.imwrite(filename, display_image)
            rospy.loginfo(f"💾 Frame saved: {filename}")
    
    def create_display_image(self, detected, confidence, cross_center):
        """Create display image with debug info"""
        if self.controller.latest_image is None:
            return np.zeros((300, 300, 3), dtype=np.uint8)
        
        # Get debug frame from controller
        debug_frame = self.controller.draw_debug_info(self.controller.latest_image)
        if debug_frame is None:
            debug_frame = self.controller.latest_image.copy()
        
        # Add additional info
        current_time = rospy.get_time()
        
        # Detection status
        status_color = (0, 255, 0) if detected else (0, 255, 255)
        status_text = f"DETECTED: {confidence}" if detected else f"NOT DETECTED: {confidence}"
        cv2.putText(debug_frame, status_text, (10, 60), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, status_color, 1)
        
        # Frame counter
        counter_text = f"Frame: {self.total_frames}"
        cv2.putText(debug_frame, counter_text, (10, 80), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
        
        # Detection rate
        if self.total_frames > 0:
            rate = (self.detection_count / self.total_frames) * 100
            rate_text = f"Rate: {rate:.1f}%"
            cv2.putText(debug_frame, rate_text, (10, 100), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
        
        # Recent detection indicator
        time_since_detection = current_time - self.last_detection_time
        if time_since_detection < 2.0:  # Within last 2 seconds
            indicator_color = (0, 255, 0)  # Green
            cv2.circle(debug_frame, (280, 30), 8, indicator_color, -1)
        
        # Show detection center if available
        if detected and cross_center is not None:
            cross_detection_roi_y = int(self.controller.HEIGHT * self.controller.CROSS_DETECTION_ROI_Y_PERCENT)
            cross_detection_roi_h = int(self.controller.HEIGHT * self.controller.CROSS_DETECTION_ROI_H_PERCENT)
            
            # Draw center point
            cv2.circle(debug_frame, (cross_center, cross_detection_roi_y + cross_detection_roi_h//2), 
                      5, (0, 0, 255), -1)
        
        return debug_frame
    
    def print_summary(self):
        """Print final summary"""
        if self.total_frames == 0:
            return
            
        detection_rate = (self.detection_count / self.total_frames) * 100
        
        print("\n" + "=" * 50)
        print("CROSS DETECTION MONITOR SUMMARY")
        print("=" * 50)
        print(f"Total Frames Processed: {self.total_frames}")
        print(f"Detections: {self.detection_count}")
        print(f"Detection Rate: {detection_rate:.2f}%")
        
        # Analyze detection pattern
        if len(self.detection_history) > 10:
            recent_detections = [d for d in self.detection_history[-50:] if d['detected']]
            if recent_detections:
                avg_confidence = {}
                for d in recent_detections:
                    conf = d['confidence']
                    if conf not in avg_confidence:
                        avg_confidence[conf] = 0
                    avg_confidence[conf] += 1
                
                print("\nConfidence Distribution (last 50 detections):")
                for conf, count in avg_confidence.items():
                    print(f"  {conf}: {count} times")
        
        print("=" * 50)
    
    def run(self):
        """Main run loop"""
        try:
            rospy.spin()
        except rospy.ROSInterruptException:
            pass
        finally:
            self.print_summary()
            cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        monitor = CrossDetectionMonitor()
        monitor.run()
    except Exception as e:
        rospy.logerr(f"Monitor error: {e}")
        import traceback
        traceback.print_exc()