#!/bin/bash
# Test all Nora expressions

source /opt/ros/humble/setup.bash
source ~/nora_ws/install/setup.bash

EXPRESSIONS=("neutral" "happy" "sad" "curious" "thinking" "alert" "listening" "speaking" "sleeping" "tired" "error")

echo "Testing Nora expressions..."
echo "Press Ctrl+C to stop"
echo ""

for expr in "${EXPRESSIONS[@]}"; do
    echo "Expression: $expr"
    ros2 topic pub --once /expression/command nora_head/msg/ExpressionCommand "{expression: '$expr'}" > /dev/null 2>&1
    sleep 3
done

echo ""
echo "Test complete!"
