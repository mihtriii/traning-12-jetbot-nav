#!/bin/bash

# Script to run the simple angle calculator

echo "🚀 Starting Simple Angle Calculator..."

# Check if ROS is running
if ! pgrep -x "roscore" > /dev/null; then
    echo "⚠️  Warning: roscore is not running!"
    echo "   Please start roscore in another terminal first:"
    echo "   $ roscore"
    exit 1
fi

# Check if required topics are available
echo "📡 Checking for required ROS topics..."

if ! rostopic list | grep -q "/scan"; then
    echo "⚠️  Warning: /scan topic not found!"
fi

if ! rostopic list | grep -q "/csi_cam_0/image_raw"; then
    echo "⚠️  Warning: /csi_cam_0/image_raw topic not found!"
fi

echo "✅ Starting angle calculator..."

# Run the angle calculator
cd "$(dirname "$0")" || exit 1
python3 angle_calculator.py