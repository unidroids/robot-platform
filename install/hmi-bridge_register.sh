#!/bin/bash

SERVICE_FILE="/etc/systemd/system/robot-hmi-bridge.service"
LOG_DIR="/data/logs/hmi-bridge"
LOG_FILE="$LOG_DIR/hmi-bridge.log"

echo "📁 Vytvářím logovací složku..."
sudo mkdir -p "$LOG_DIR"
sudo touch "$LOG_FILE"
sudo chmod 664 "$LOG_FILE"

echo "🛠️ Vytvářím systemd službu: robot-hmi-bridge.service"

sudo tee "$SERVICE_FILE" > /dev/null <<EOF
[Unit]
Description=Robotour 2025 – HMI Bridge service
After=network.target

[Service]
User=user
WorkingDirectory=/opt/projects/robotour/hmi-bridge

# vlastní spuštění (un-buffer mód kvůli okamžitému logování)
Environment=PYTHONUNBUFFERED=1

# před spuštěním ukonči libovolný proces, který drží port 9020
ExecStartPre=/bin/bash -c '/usr/bin/fuser -k 9020/tcp || true'
ExecStartPre=/bin/sleep 0.5

ExecStart=/usr/bin/python3 main.py

# logujeme přes systemd přesměrování
StandardOutput=append:/data/logs/hmi-bridge/hmi-bridge.log
StandardError=append:/data/logs/hmi-bridge/hmi-bridge.log

Restart=always
RestartSec=2

[Install]
WantedBy=multi-user.target
EOF

echo "🔁 Aktivuji službu robot-hmi-bridge.service"
sudo systemctl daemon-reexec
sudo systemctl daemon-reload
sudo systemctl enable --now robot-hmi-bridge.service
echo "   tail -f $LOG_FILE"
