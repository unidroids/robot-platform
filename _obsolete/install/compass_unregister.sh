#!/bin/bash
set -e

echo "Stopping and disabling service..."
sudo systemctl stop robot-compass.service || true
sudo systemctl disable robot-compass.service || true

echo "Removing service file..."
sudo rm -f /etc/systemd/system/robot-compass.service

echo "Reloading systemd..."
sudo systemctl daemon-reexec
sudo systemctl daemon-reload

echo "✅ robot-compass unregistered."
