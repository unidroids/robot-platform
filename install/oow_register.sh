#!/bin/bash
set -euo pipefail
SERVICE_FILE="/etc/systemd/system/robot-oow.service"
LOG_DIR="/data/logs/oow"
VENV_BIN="/robot/opt/projects/robotour/venv-robotour/bin"
WORK_DIR="/opt/projects/robotour/oow"

echo "📁 Vytvářím logovací složku..."
sudo mkdir -p "$LOG_DIR"
sudo chown -R user:user "$LOG_DIR"

echo "📦 Instalace závislostí do venv"
sudo -u user "$VENV_BIN/pip" install --upgrade pip >/dev/null || true
sudo -u user "$VENV_BIN/pip" install -r "$WORK_DIR/requirements.txt"

cat <<EOF | sudo tee "$SERVICE_FILE" > /dev/null
[Unit]
Description=Robotour 2025 – oow server (TCP 9013)
After=network.target bluetooth.service
BindsTo=bluetooth.service
StartLimitIntervalSec=0

[Service]
User=user
WorkingDirectory=$WORK_DIR
Environment=PYTHONUNBUFFERED=1
ExecStartPre=/bin/bash -c '/usr/bin/fuser -k 9013/tcp || true'
ExecStartPre=/bin/sleep 0.5
ExecStart=$VENV_BIN/python $WORK_DIR/main.py
StandardOutput=append:$LOG_DIR/service.log
StandardError=append:$LOG_DIR/service.log
Restart=always
RestartSec=5

[Install]
WantedBy=multi-user.target
EOF

echo "🔄 Načítám systemd..."
sudo systemctl daemon-reexec
sudo systemctl daemon-reload
echo "▶️ Povoluji a spouštím službu..."
sudo systemctl enable --now robot-oow.service

sleep 0.3
sudo systemctl --no-pager --full status robot-oow.service || true

echo "   tail -f $LOG_DIR/service.log"
