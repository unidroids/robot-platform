#!/bin/bash
set -e

echo "Stopping and disabling service..."
sudo systemctl stop robot-maps.service || true
sudo systemctl disable robot-maps.service || true

echo "Removing service file..."
sudo rm -f /etc/systemd/system/robot-maps.service

echo "Reloading systemd..."
sudo systemctl daemon-reexec
sudo systemctl daemon-reload

echo "✅ robot-maps unregistered."
