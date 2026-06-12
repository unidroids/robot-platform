#!/bin/bash

SERVICE_FILE="/etc/systemd/system/robot-logger.service"
LOG_DIR="/data/logs/logger"
LOG_FILE="$LOG_DIR/logger.log"

echo "📁 Vytvářím logovací složku..."
sudo mkdir -p "$LOG_DIR"
sudo touch "$LOG_FILE"
sudo chmod 664 "$LOG_FILE"

echo "🛠️ Vytvářím systemd službu: robot-logger.service"

sudo tee "$SERVICE_FILE" > /dev/null <<EOF
[Unit]
Description=Robotour 2025 – logger socket server
After=network.target

[Service]
User=user
WorkingDirectory=/opt/projects/robotour/logger

# vlastní spuštění (un-buffer mód kvůli okamžitému logování)
Environment=PYTHONUNBUFFERED=1

# před spuštěním ukonči libovolný proces, který drží port 9012
ExecStartPre=/bin/bash -c '/usr/bin/fuser -k 9012/tcp || true'
ExecStartPre=/bin/sleep 0.5

ExecStart=/usr/bin/python3 main.py

# logujeme přes systemd přesměrování
StandardOutput=append:/data/logs/logger/logger.log
StandardError=append:/data/logs/logger/logger.log

Restart=always
RestartSec=2

[Install]
WantedBy=multi-user.target
EOF

echo "🔁 Aktivuji službu robot-logger.service"
sudo systemctl daemon-reexec
sudo systemctl daemon-reload
sudo systemctl enable --now robot-logger.service
echo "   tail -f $LOG_FILE"
