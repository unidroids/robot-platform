#!/bin/bash

echo "🛑 Zastavuji a odstraňuji robot-camera-stereo.service..."
sudo systemctl stop robot-camera-stereo.service
sudo systemctl disable robot-camera-stereo.service
sudo rm -f /etc/systemd/system/robot-camera-stereo.service
sudo systemctl daemon-reload
