#!/bin/bash

# Script to run the simple angle calculator

echo "🚀 Starting Simple Angle Calculator..."

# Check for mock mode flag
MOCK_MODE=false
if [[ "$1" == "--mock" ]] || [[ "$1" == "--test" ]]; then
    MOCK_MODE=true
    echo "🔧 Mock mode enabled - using simulated data"
fi

# Check if ROS is running (only if not mock mode)
if [[ "$MOCK_MODE" == false ]]; then
    if ! pgrep -x "roscore" > /dev/null; then
        echo "⚠️  Warning: roscore is not running!"
        echo "   Please start roscore in another terminal first:"
        echo "   $ roscore"
        echo ""
        echo "   Or run with mock data:"
        echo "   $ $0 --mock"
        exit 1
    fi

    # Check if required topics are available
    echo "📡 Checking for required ROS topics..."

    if ! rostopic list | grep -q "/scan"; then
        echo "⚠️  Warning: /scan topic not found!"
        echo "   You can run with mock data: $0 --mock"
    fi

    if ! rostopic list | grep -q "/csi_cam_0/image_raw"; then
        echo "⚠️  Warning: /csi_cam_0/image_raw topic not found!"
        echo "   You can run with mock data: $0 --mock"
    fi
fi

echo "✅ Starting angle calculator..."

# Run the angle calculator
cd "$(dirname "$0")" || exit 1

if [[ "$MOCK_MODE" == true ]]; then
    python3 angle_calculator.py --mock
else
    python3 angle_calculator.py
fi