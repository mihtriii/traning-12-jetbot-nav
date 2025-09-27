#!/bin/bash

# Script to test diagonal detection

echo "🔧 Testing Diagonal Detection System..."

# Check command line arguments
MODE="continuous"
INTERVAL=3.0

while [[ $# -gt 0 ]]; do
    case $1 in
        --mock)
            MODE="mock"
            shift
            ;;
        --single)
            MODE="single"
            shift
            ;;
        --continuous)
            MODE="continuous"
            shift
            ;;
        --interval=*)
            INTERVAL="${1#*=}"
            shift
            ;;
        -h|--help)
            echo "Usage: $0 [OPTIONS]"
            echo ""
            echo "Test diagonal detection (only objects near 45°, 135°, 225°, 315°)"
            echo ""
            echo "Options:"
            echo "  --mock              Test with mock data (no real sensor needed)"
            echo "  --single            Run single detection test"
            echo "  --continuous        Run continuous monitoring (default)"
            echo "  --interval=N        Set monitoring interval (seconds, default: 3.0)"
            echo "  -h, --help          Show this help message"
            echo ""
            echo "Examples:"
            echo "  $0 --mock                    # Test with simulated data"
            echo "  $0 --single                  # Single detection test"
            echo "  $0 --continuous --interval=2 # Monitor every 2 seconds"
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            echo "Use --help for usage information"
            exit 1
            ;;
    esac
done

echo "🎯 Detection Mode: Only objects near diagonal axes (45°, 135°, 225°, 315°)"

if [[ "$MODE" == "mock" ]]; then
    echo "🔧 Running with mock data (no ROS required)"
    echo ""
    cd "$(dirname "$0")" || exit 1
    python3 test_diagonal_detection.py --mock
    exit 0
fi

# Check if ROS is running for real sensor tests
if ! pgrep -x "roscore" > /dev/null; then
    echo "❌ roscore is not running!"
    echo "   For real sensor test, start roscore first: roscore"
    echo "   Or run with mock data: $0 --mock"
    exit 1
fi

echo "✅ roscore is running"

# Check if /scan topic exists
if ! rostopic list | grep -q "/scan"; then
    echo "❌ /scan topic NOT found"
    echo "   Run with mock data: $0 --mock"
    exit 1
fi

echo "✅ /scan topic found"

# Run the test
cd "$(dirname "$0")" || exit 1

case $MODE in
    "single")
        echo "🔍 Running single diagonal detection test..."
        python3 test_diagonal_detection.py --single
        ;;
    "continuous")
        echo "🔄 Running continuous diagonal detection (interval: ${INTERVAL}s)"
        echo "   Press Ctrl+C to stop"
        python3 test_diagonal_detection.py --interval=$INTERVAL
        ;;
    *)
        echo "❌ Unknown mode: $MODE"
        exit 1
        ;;
esac