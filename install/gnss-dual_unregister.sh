#!/bin/bash
set -e

echo "Stopping and disabling service..."
sudo systemctl stop robot-gnss-dual.service || true
sudo systemctl disable robot-gnss-dual.service || true

echo "Removing service file..."
sudo rm -f /etc/systemd/system/robot-gnss-dual.service

echo "Reloading systemd..."
sudo systemctl daemon-reexec
sudo systemctl daemon-reload

echo "✅ robot-gnss-dual unregistered."
