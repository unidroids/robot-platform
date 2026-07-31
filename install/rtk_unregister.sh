#!/bin/bash
set -e

echo "Stopping and disabling service..."
sudo systemctl stop robot-rtk.service || true
sudo systemctl disable robot-rtk.service || true

echo "Removing service file..."
sudo rm -f /etc/systemd/system/robot-rtk.service

echo "Reloading systemd..."
sudo systemctl daemon-reexec
sudo systemctl daemon-reload

echo "✅ robot-rtk unregistered."
