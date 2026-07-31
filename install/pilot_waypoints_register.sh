#!/bin/bash
set -e

# ==========================================
# 1. NASTAVENÍ PROMĚNNÝCH
# ==========================================
PROJECT_DIR="/opt/projects/robotour"
SERVICE_DIR="$PROJECT_DIR/pilot_waypoints"

SERVICE_NAME="robot-pilot-waypoints.service"
SERVICE_PATH="/etc/systemd/system/$SERVICE_NAME"
LOG_DIR="/data/logs/pilot_waypoints"
LOG_FILE="$LOG_DIR/pilot_waypoints.log"

# ==========================================
# 2. PŘÍPRAVA ADRESÁŘŮ A PRÁV
# ==========================================
echo "📁 Vytvářím logovací složku ($LOG_DIR)..."
sudo mkdir -p "$LOG_DIR"
sudo touch "$LOG_FILE"
sudo chmod 664 "$LOG_FILE"

# ==========================================
# 3. VYTVOŘENÍ SYSTEMD SLUŽBY
# ==========================================
echo "🛠️ Vytvářím systemd službu: $SERVICE_NAME"

sudo tee "$SERVICE_PATH" > /dev/null <<EOF
[Unit]
Description=Robotour 2025 - Pilot Waypoints (Python)
After=network.target

[Service]
User=user
WorkingDirectory=$SERVICE_DIR

Environment=PYTHONUNBUFFERED=1

# Před spuštěním ukonči libovolný proces, který drží port 9101
ExecStartPre=/bin/bash -c '/usr/bin/fuser -k 9101/tcp || true'
ExecStartPre=/bin/sleep 0.5

ExecStart=/robot/opt/projects/robotour/venv-robotour/bin/python $SERVICE_DIR/main.py

StandardOutput=append:$LOG_FILE
StandardError=append:$LOG_FILE

Restart=always
RestartSec=3
NoNewPrivileges=true

[Install]
WantedBy=multi-user.target
EOF

# ==========================================
# 4. AKTIVACE A START
# ==========================================
echo "🔄 Aktualizuji systemd a aktivuji službu..."
sudo systemctl daemon-reexec
sudo systemctl daemon-reload
sudo systemctl enable --now "$SERVICE_NAME"

echo "✅ Služba $SERVICE_NAME úspěšně nasazena a spuštěna!"
echo "   Pro sledování logu zadejte: tail -f $LOG_FILE"
