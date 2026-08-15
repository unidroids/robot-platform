#!/bin/bash
set -euo pipefail
SERVICE_FILE="/etc/systemd/system/robot-pilot-manual.service"
LOG_DIR="/data/logs/pilot_manual"
VENV_BIN="/robot/opt/projects/robotour/venv-robotour/bin"
WORK_DIR="/opt/projects/robotour/pilot_manual"

echo "📁 Vytvářím logovací složku..."
sudo mkdir -p "$LOG_DIR"
sudo chown -R user:user "$LOG_DIR"

cat <<EOF | sudo tee "$SERVICE_FILE" > /dev/null
[Unit]
Description=Robotour 2025 – server-pilot-manual (TCP 9103)
After=network.target

[Service]
User=user
WorkingDirectory=$WORK_DIR
Environment=PYTHONUNBUFFERED=1
ExecStartPre=/bin/bash -c '/usr/bin/fuser -k 9103/tcp || true'
ExecStartPre=/bin/sleep 0.5
ExecStart=$VENV_BIN/python $WORK_DIR/main.py
StandardOutput=append:$LOG_DIR/service.log
StandardError=append:$LOG_DIR/service.log
Restart=always
RestartSec=1

[Install]
WantedBy=multi-user.target
EOF

echo "🔄 Načítám systemd..."
sudo systemctl daemon-reexec
sudo systemctl daemon-reload
echo "▶️ Povoluji a spouštím službu..."
sudo systemctl enable --now robot-pilot-manual.service

sleep 0.3
sudo systemctl --no-pager --full status robot-pilot-manual.service || true

echo "   tail -f $LOG_DIR/service.log"
