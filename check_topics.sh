#!/bin/bash

# Script to check ROS topics and debug angle calculator

echo "🔍 Checking ROS environment..."

# Check if ROS is running
if ! pgrep -x "roscore" > /dev/null; then
    echo "❌ roscore is not running!"
    echo "   Start roscore first: roscore"
    exit 1
fi

echo "✅ roscore is running"

# List all available topics
echo -e "\n📡 Available ROS Topics:"
echo "========================"
rostopic list

# Check specific topics we need
echo -e "\n🎯 Checking required topics:"
echo "============================="

if rostopic list | grep -q "/scan"; then
    echo "✅ /scan topic found"
    echo "   Topic info:"
    rostopic info /scan | sed 's/^/   /'
else
    echo "❌ /scan topic NOT found"
fi

if rostopic list | grep -q "/csi_cam_0/image_raw"; then
    echo "✅ /csi_cam_0/image_raw topic found" 
    echo "   Topic info:"
    rostopic info /csi_cam_0/image_raw | sed 's/^/   /'
else
    echo "❌ /csi_cam_0/image_raw topic NOT found"
    
    # Look for alternative camera topics
    echo "🔍 Looking for alternative camera topics:"
    camera_topics=$(rostopic list | grep -i "image\|camera" || echo "None found")
    echo "   $camera_topics"
fi

# Check topic rates
echo -e "\n📈 Topic rates (if publishing):"
echo "================================="

if rostopic list | grep -q "/scan"; then
    echo -n "   /scan: "
    timeout 3s rostopic hz /scan 2>/dev/null | head -1 || echo "No data received"
fi

if rostopic list | grep -q "/csi_cam_0/image_raw"; then
    echo -n "   /csi_cam_0/image_raw: "
    timeout 3s rostopic hz /csi_cam_0/image_raw 2>/dev/null | head -1 || echo "No data received"
fi

echo -e "\n🚀 Ready to run angle_calculator.py"
echo "   Run: python3 angle_calculator.py"