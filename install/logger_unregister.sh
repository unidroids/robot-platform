#!/bin/bash

echo "🛑 Zastavuji a odstraňuji robot-logger.service..."
sudo systemctl stop robot-logger.service
sudo systemctl disable robot-logger.service
sudo rm -f /etc/systemd/system/robot-logger.service
