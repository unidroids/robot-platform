#!/bin/bash
set -e

SERVICE_PATH="/etc/systemd/system/robot-compass.service"
LOG_DIR="/data/logs/compass"
LOG_FILE="$LOG_DIR/compass.log"

echo "📁 Creating log directory..."
sudo mkdir -p "$LOG_DIR"
sudo chown user:user "$LOG_DIR" || true
sudo touch "$LOG_FILE"
sudo chmod 664 "$LOG_FILE"
sudo chown user:user "$LOG_FILE" || true

echo "🛠  Creating systemd service robot-compass"

sudo tee "$SERVICE_PATH" > /dev/null <<'EOF'
[Unit]
Description=Robot Compass for Robotour
Wants=network-online.target
After=network-online.target

# Pomůže zachytit chybějící soubory srozumitelněji než CHDIR fail
ConditionPathExists=/opt/projects/robotour/compass/main.py

[Service]
User=user
WorkingDirectory=/opt/projects/robotour/compass

# před spuštěním ukonči libovolný proces, který drží port 9014
ExecStartPre=/bin/bash -c '/usr/bin/fuser -k 9014/tcp || true'
ExecStartPre=/bin/sleep 0.5

Environment=PYTHONUNBUFFERED=1

ExecStart=/robot/opt/projects/robotour/venv-robotour/bin/python /opt/projects/robotour/compass/main.py

StandardOutput=append:/data/logs/compass/compass.log
StandardError=append:/data/logs/compass/compass.log

Restart=always
RestartSec=3
NoNewPrivileges=true

[Install]
WantedBy=multi-user.target
EOF

echo "🔄 Reloading and enabling service..."
sudo systemctl daemon-reexec
sudo systemctl daemon-reload
sudo systemctl enable --now robot-compass.service

echo "✅ Service robot-compass is now active. Check logs with:"
echo "   tail -f $LOG_FILE"
