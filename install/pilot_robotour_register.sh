#!/bin/bash
set -e

# ==========================================
# 1. NASTAVENÍ PROMĚNNÝCH
# ==========================================
PROJECT_DIR="/opt/projects/robotour"
SERVICE_DIR="$PROJECT_DIR/pilot_robotour"

SERVICE_NAME="robot-pilot-robotour.service"
SERVICE_PATH="/etc/systemd/system/$SERVICE_NAME"
LOG_DIR="/data/logs/pilot_robotour"
LOG_FILE="$LOG_DIR/pilot_robotour.log"
DATA_DIR="/data/robot/pilot_robotour"

# ==========================================
# 2. PŘÍPRAVA ADRESÁŘŮ A PRÁV
# ==========================================
echo "📁 Vytvářím logovací složku ($LOG_DIR)..."
sudo mkdir -p "$LOG_DIR"
sudo chown -R user:user "$LOG_DIR" || true
sudo touch "$LOG_FILE"
sudo chmod 664 "$LOG_FILE"
sudo chown user:user "$LOG_FILE" || true

echo "📁 Vytvářím runtime data složku pro telemetrii ($DATA_DIR)..."
sudo mkdir -p "$DATA_DIR"
sudo chown -R user:user "$DATA_DIR" || true
sudo chmod -R 775 "$DATA_DIR" || true

# ==========================================
# 3. VYTVOŘENÍ SYSTEMD SLUŽBY
# ==========================================
echo "🛠️ Vytvářím systemd službu: $SERVICE_NAME"

sudo tee "$SERVICE_PATH" > /dev/null <<EOF
[Unit]
Description=Robotour 2025 - Pilot Robotour (Python)
Wants=network-online.target
After=network-online.target

# Pomůže zachytit chybějící soubory srozumitelněji než CHDIR fail
ConditionPathExists=$SERVICE_DIR/main.py

[Service]
User=user
WorkingDirectory=$SERVICE_DIR

Environment=PYTHONUNBUFFERED=1

# Před spuštěním ukonči libovolný proces, který drží port 9104
ExecStartPre=/bin/bash -c '/usr/bin/fuser -k 9104/tcp || true'
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
echo "   Pro ověření stavu: systemctl status $SERVICE_NAME"
echo "   Pro sledování logu: tail -f $LOG_FILE"
