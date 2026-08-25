#!/bin/bash

echo "🛑 Zastavuji a odstraňuji robot-hmi-bridge.service..."
sudo systemctl stop robot-hmi-bridge.service
sudo systemctl disable robot-hmi-bridge.service
sudo rm -f /etc/systemd/system/robot-hmi-bridge.service
sudo systemctl daemon-reload
