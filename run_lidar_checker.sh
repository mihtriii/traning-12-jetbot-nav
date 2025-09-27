#!/bin/bash

# Script to run the LiDAR checker

echo "🔍 Starting LiDAR Checker..."

# Check command line arguments
CONTINUOUS=false
INTERVAL=2.0

while [[ $# -gt 0 ]]; do
    case $1 in
        -c|--continuous)
            CONTINUOUS=true
            shift
            ;;
        --interval=*)
            INTERVAL="${1#*=}"
            shift
            ;;
        -h|--help)
            echo "Usage: $0 [OPTIONS]"
            echo ""
            echo "Options:"
            echo "  -c, --continuous    Run in continuous monitoring mode"
            echo "  --interval=N        Set monitoring interval (seconds, default: 2.0)"
            echo "  -h, --help          Show this help message"
            echo ""
            echo "Examples:"
            echo "  $0                  # Single check"
            echo "  $0 -c               # Continuous monitoring every 2s"
            echo "  $0 -c --interval=1  # Continuous monitoring every 1s"
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            echo "Use --help for usage information"
            exit 1
            ;;
    esac
done

# Check if ROS is running
if ! pgrep -x "roscore" > /dev/null; then
    echo "❌ roscore is not running!"
    echo "   Please start roscore first:"
    echo "   $ roscore"
    exit 1
fi

echo "✅ roscore is running"

# Check if /scan topic exists
echo "📡 Checking for /scan topic..."
if rostopic list | grep -q "/scan"; then
    echo "✅ /scan topic found"
    
    # Check if topic is publishing
    echo "📊 Checking topic activity..."
    echo -n "   /scan: "
    timeout 3s rostopic hz /scan 2>/dev/null | head -1 || echo "No data received in 3s"
    
else
    echo "❌ /scan topic NOT found"
    echo "   Available topics:"
    rostopic list | grep -E "(scan|laser|lidar)" | sed 's/^/   /' || echo "   No LiDAR-related topics found"
    exit 1
fi

echo ""
echo "🚀 Starting LiDAR checker..."

# Run the checker
cd "$(dirname "$0")" || exit 1

if [[ "$CONTINUOUS" == true ]]; then
    echo "🔄 Running in continuous mode (interval: ${INTERVAL}s)"
    echo "   Press Ctrl+C to stop"
    python3 lidar_checker.py --continuous --interval=$INTERVAL
else
    echo "🔍 Running single check"
    python3 lidar_checker.py
fi