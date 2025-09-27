#!/bin/bash

echo "🚀 Khởi chạy Line Angle Analyzer..."
echo "📊 Phân tích góc hợp bởi line từ Camera ROS và Cluster từ LiDAR"
echo "🔍 Nhấn Ctrl+C để dừng"
echo "="*60

# Đảm bảo ROS environment đã được setup
if [ -z "$ROS_MASTER_URI" ]; then
    echo "⚠️  ROS environment chưa được setup!"
    echo "💡 Chạy: source /opt/ros/noetic/setup.bash"
    echo "   hoặc: source ~/catkin_ws/devel/setup.bash"
    exit 1
fi

cd /Users/mihtriii/Desktop/jetbot

# Chạy line angle analyzer
python3 line_angle_analyzer.py