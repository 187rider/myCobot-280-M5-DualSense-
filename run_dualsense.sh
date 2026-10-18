#!/bin/bash
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$DIR"

echo "Starting myCobot PS5 DualSense Control..."
.venv/bin/python dualsense_control.py
