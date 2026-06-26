#!/bin/bash

echo "Stopping and disabling service..."
sudo systemctl stop robot-drive-crawler.service
sudo systemctl disable robot-drive-crawler.service

echo "Removing service file..."
sudo rm -f /etc/systemd/system/robot-drive-crawler.service

echo "Reloading systemd..."
sudo systemctl daemon-reexec
sudo systemctl daemon-reload
