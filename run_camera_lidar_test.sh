#!/bin/bash

# Script to test Camera-LiDAR Intersection Detection

echo "🔧 Testing Camera-LiDAR Intersection Detection System..."

# Check command line arguments
VISUALIZATION=false

while [[ $# -gt 0 ]]; do
    case $1 in
        --viz|--visual)
            VISUALIZATION=true
            shift
            ;;
        -h|--help)
            echo "Usage: $0 [OPTIONS]"
            echo ""
            echo "Test camera-first intersection detection with LiDAR confirmation"
            echo ""
            echo "Options:"
            echo "  --viz, --visual     Show visualization window"
            echo "  -h, --help          Show this help message"
            echo ""
            echo "How it works:"
            echo "  1. Camera detects cross-line intersection first"
            echo "  2. System waits for LiDAR confirmation"
            echo "  3. Only triggers intersection handling when both agree"
            echo ""
            echo "Examples:"
            echo "  $0                  # Test without visualization"
            echo "  $0 --viz            # Test with visual feedback"
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            echo "Use --help for usage information"
            exit 1
            ;;
    esac
done

echo "🎯 Detection Strategy: Camera detects → LiDAR confirms → Action"

# Check if ROS is running
if ! pgrep -x "roscore" > /dev/null; then
    echo "❌ roscore is not running!"
    echo "   Please start roscore in another terminal first:"
    echo "   $ roscore"
    exit 1
fi

echo "✅ roscore is running"

# Check if required topics exist
echo "📡 Checking for required ROS topics..."

missing_topics=0

if ! rostopic list | grep -q "/scan"; then
    echo "❌ /scan topic NOT found"
    missing_topics=$((missing_topics + 1))
else
    echo "✅ /scan topic found"
fi

if ! rostopic list | grep -q "/csi_cam_0/image_raw"; then
    echo "❌ /csi_cam_0/image_raw topic NOT found"
    missing_topics=$((missing_topics + 1))
else
    echo "✅ /csi_cam_0/image_raw topic found"
fi

if [[ $missing_topics -gt 0 ]]; then
    echo ""
    echo "⚠️  Missing $missing_topics required topics"
    echo "   Make sure camera and LiDAR nodes are running"
    exit 1
fi

# Check topic rates
echo ""
echo "📈 Checking topic activity..."

echo -n "   /scan: "
timeout 3s rostopic hz /scan 2>/dev/null | head -1 || echo "No data received in 3s"

echo -n "   /csi_cam_0/image_raw: "
timeout 3s rostopic hz /csi_cam_0/image_raw 2>/dev/null | head -1 || echo "No data received in 3s"

echo ""
echo "🚀 Starting Camera-LiDAR Intersection Detection Test..."

if [[ "$VISUALIZATION" == true ]]; then
    echo "👁️  Visualization enabled - press 'q' in image window to quit"
fi

echo "📋 Test sequence:"
echo "   1. Move robot to approach intersection"
echo "   2. Watch for camera detection (yellow ROI)"
echo "   3. System waits for LiDAR confirmation"
echo "   4. Both sensors must agree to trigger action"
echo ""
echo "Press Ctrl+C to stop test"

# Run the test
cd "$(dirname "$0")" || exit 1

if [[ "$VISUALIZATION" == true ]]; then
    python3 test_camera_lidar_intersection.py --viz
else
    python3 test_camera_lidar_intersection.py
fi