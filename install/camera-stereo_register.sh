#!/bin/bash

SERVICE_FILE="/etc/systemd/system/robot-camera-stereo.service"
LOG_DIR="/data/logs/camera-stereo"
LOG_FILE="$LOG_DIR/camera-stereo.log"

echo "📁 Vytvářím logovací složku pro camera-stereo..."
sudo mkdir -p "$LOG_DIR"
sudo touch "$LOG_FILE"
sudo chmod 664 "$LOG_FILE"

echo "🛠️ Vytvářím systemd službu: robot-camera-stereo.service"

sudo tee "$SERVICE_FILE" > /dev/null <<EOF
[Unit]
Description=Robotour 2025 – kamera stereo socket server
After=network.target

[Service]
User=user
WorkingDirectory=/opt/projects/robotour/camera-stereo

# vlastní spuštění (un-buffer mód kvůli okamžitému logování)
Environment=PYTHONUNBUFFERED=1

# před spuštěním ukonči libovolný proces, který drží port 9001
ExecStartPre=/bin/bash -c '/usr/bin/fuser -k 9001/tcp || true'
ExecStartPre=/bin/sleep 0.5

ExecStart=/usr/bin/python3 main.py

# logujeme přes systemd přesměrování
StandardOutput=append:/data/logs/camera-stereo/camera-stereo.log
StandardError=append:/data/logs/camera-stereo/camera-stereo.log

Restart=always
RestartSec=2

[Install]
WantedBy=multi-user.target
EOF

echo "🔁 Aktivuji službu robot-camera-stereo.service"
sudo systemctl daemon-reexec
sudo systemctl daemon-reload
sudo systemctl enable --now robot-camera-stereo.service
echo "   tail -f $LOG_FILE"
