#!/bin/bash

echo "🔍 Cross Detection Improvement Tools"
echo "===================================="
echo ""
echo "Available tools:"
echo ""
echo "1. 🧪 Test Cross Detection Sensitivity"
echo "   - Tests detection with different cross types"
echo "   - Identifies missed detections"
echo "   - Generates debug images"
echo ""
echo "2. 📊 Real-time Detection Monitor (requires ROS)"
echo "   - Live monitoring of detection performance"
echo "   - Visual debug with ROI overlay"
echo "   - Statistics tracking"
echo ""
echo "3. ⚙️  Check Current Parameters"
echo "   - Display current detection parameters"
echo "   - Compare with recommended values"
echo ""

read -p "Select tool (1-3) or 'q' to quit: " choice

case $choice in
    1)
        echo "🧪 Running Cross Detection Sensitivity Test..."
        python3 test_cross_detection_sensitivity.py
        ;;
    2)
        echo "📊 Starting Real-time Detection Monitor..."
        echo "Make sure ROS and camera are running!"
        echo "roslaunch jetbot_ros jetbot.launch"
        echo ""
        read -p "Press Enter when ready or Ctrl+C to cancel..."
        python3 monitor_cross_detection.py
        ;;
    3)
        echo "⚙️  Current Cross Detection Parameters:"
        echo "======================================"
        grep -A 10 "Parameters cho Camera-LiDAR" ros_lidar_follower.py | grep -E "(CROSS_|ROI_)" || echo "Parameters not found"
        echo ""
        echo "💡 Recommended settings for high sensitivity:"
        echo "   CROSS_MIN_ASPECT_RATIO = 1.5   (lower = more sensitive)"
        echo "   CROSS_MIN_WIDTH_RATIO = 0.25   (lower = detect smaller crosses)"
        echo "   CROSS_MAX_HEIGHT_RATIO = 0.9   (higher = allow taller crosses)"
        echo "   CROSS_DETECTION_ROI_Y_PERCENT = 0.45  (earlier detection)"
        echo "   CROSS_DETECTION_ROI_H_PERCENT = 0.30  (larger detection area)"
        ;;
    q|Q)
        echo "👋 Goodbye!"
        exit 0
        ;;
    *)
        echo "❌ Invalid choice. Please select 1-3 or 'q'"
        ;;
esac

echo ""
echo "✅ Done!"
echo ""
echo "💡 Tips for reducing missed detections:"
echo "   - Ensure good lighting conditions"
echo "   - Check line color calibration (HSV range)"
echo "   - Adjust ROI position if intersections appear at different heights"
echo "   - Consider lowering confidence thresholds for more sensitive detection"
echo "   - Use temporal smoothing to reduce false negatives"
echo ""
echo "🔧 If still missing detections, try:"
echo "   - Decrease CROSS_MIN_ASPECT_RATIO to 1.2"
echo "   - Decrease CROSS_MIN_WIDTH_RATIO to 0.2" 
echo "   - Increase CROSS_DETECTION_ROI_H_PERCENT to 0.35"
echo "   - Add more morphological operations in detect_camera_intersection()"