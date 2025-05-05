#!/bin/bash

echo "Checking if anything is running on port 5000..."
PID=$(lsof -ti:5000)
if [ ! -z "$PID" ]; then
    echo "Killing old server process PID=$PID..."
    kill -9 $PID
fi

echo "Starting Flask server..."
python3 web_joystick_server_mac.py

