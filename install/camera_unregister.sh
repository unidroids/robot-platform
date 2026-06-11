#!/bin/bash

echo "🛑 Zastavuji a odstraňuji robot-camera.service..."
sudo systemctl stop robot-camera.service
sudo systemctl disable robot-camera.service
sudo rm -f /etc/systemd/system/robot-camera.service
