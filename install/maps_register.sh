#!/bin/bash
set -e

SERVICE_PATH="/etc/systemd/system/robot-maps.service"
LOG_DIR="/data/logs/maps"
LOG_FILE="$LOG_DIR/maps.log"

echo "📁 Creating log directory..."
sudo mkdir -p "$LOG_DIR"
sudo chown user:user "$LOG_DIR" || true
sudo touch "$LOG_FILE"
sudo chmod 664 "$LOG_FILE"
sudo chown user:user "$LOG_FILE" || true

echo "📁 Creating maps runtime directory..."
sudo mkdir -p "/data/robot/maps"
sudo chown -R user:user "/data/robot/maps" || true
sudo chmod -R 775 "/data/robot/maps" || true

echo "🛠  Creating systemd service robot-maps"

sudo tee "$SERVICE_PATH" > /dev/null <<'UNIT_EOF'
[Unit]
Description=Robot Maps service for Robotour
Wants=network-online.target
After=network-online.target

# Pomůže zachytit chybějící soubory srozumitelněji než CHDIR fail
ConditionPathExists=/opt/projects/robotour/maps/main.py

[Service]
User=user
WorkingDirectory=/opt/projects/robotour/maps

# před spuštěním ukonči libovolný proces, který drží port 9040
ExecStartPre=/bin/bash -c '/usr/bin/fuser -k 9040/tcp || true'
ExecStartPre=/bin/sleep 0.5

Environment=PYTHONUNBUFFERED=1
ExecStart=/robot/opt/projects/robotour/venv-robotour/bin/python /opt/projects/robotour/maps/main.py

StandardOutput=append:/data/logs/maps/maps.log
StandardError=append:/data/logs/maps/maps.log

Restart=always
RestartSec=3
NoNewPrivileges=true

[Install]
WantedBy=multi-user.target
UNIT_EOF

echo "🔄 Reloading and enabling service..."
sudo systemctl daemon-reexec
sudo systemctl daemon-reload
sudo systemctl enable --now robot-maps.service

echo "✅ Service robot-maps is now active. Check logs with:"
echo "   tail -f $LOG_FILE"
