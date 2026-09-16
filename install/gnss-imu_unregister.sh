#!/bin/bash
set -e

echo "Stopping and disabling service..."
sudo systemctl stop robot-gnss-imu.service || true
sudo systemctl disable robot-gnss-imu.service || true

echo "Removing service file..."
sudo rm -f /etc/systemd/system/robot-gnss-imu.service

echo "Reloading systemd..."
sudo systemctl daemon-reexec
sudo systemctl daemon-reload

echo "✅ robot-gnss-imu unregistered."
