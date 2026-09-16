#!/bin/bash
set -e

SERVICE_PATH="/etc/systemd/system/robot-gnss-imu.service"
LOG_DIR="/robot/data/logs/gnss-imu"
LOG_FILE="$LOG_DIR/gnss-imu.log"

echo "📁 Creating log directory..."
sudo mkdir -p "$LOG_DIR"
sudo touch "$LOG_FILE"
sudo chmod 664 "$LOG_FILE"

echo "🛠  Creating systemd service robot-gnss-imu"

sudo tee "$SERVICE_PATH" > /dev/null <<'EOF'
[Unit]
Description=Robot GNSS-IMU server for Robotour (ESF_RAW 100Hz IMU Light Fusion)
After=network.target

[Service]
User=user
WorkingDirectory=/opt/projects/robotour/gnss-imu

# před spuštěním ukonči libovolný proces, který drží port 9016
ExecStartPre=/bin/bash -c '/usr/bin/fuser -k 9016/tcp || true'
ExecStartPre=/bin/sleep 0.5

Environment=PYTHONUNBUFFERED=1
ExecStart=/robot/opt/projects/robotour/venv-robotour/bin/python /opt/projects/robotour/gnss-imu/main.py

StandardOutput=append:/robot/data/logs/gnss-imu/gnss-imu.log
StandardError=append:/robot/data/logs/gnss-imu/gnss-imu.log

Restart=always
RestartSec=3

[Install]
WantedBy=multi-user.target
EOF

echo "🔄 Reloading and enabling service..."
sudo systemctl daemon-reexec
sudo systemctl daemon-reload
sudo systemctl enable --now robot-gnss-imu.service

echo "✅ Service robot-gnss-imu is now active. Check logs with:"
echo "   tail -f $LOG_FILE"
