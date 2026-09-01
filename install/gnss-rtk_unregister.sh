#!/bin/bash
set -e

echo "Stopping and disabling service..."
sudo systemctl stop robot-gnss-rtk.service || true
sudo systemctl disable robot-gnss-rtk.service || true

echo "Removing service file..."
sudo rm -f /etc/systemd/system/robot-gnss-rtk.service

echo "Reloading systemd..."
sudo systemctl daemon-reexec
sudo systemctl daemon-reload

echo "✅ robot-gnss-rtk unregistered."
