#!/bin/bash
set -e

SERVICE_NAME="robot-pilot-vision.service"
SERVICE_PATH="/etc/systemd/system/$SERVICE_NAME"

echo "🛑 Zastavuji službu $SERVICE_NAME..."
sudo systemctl stop "$SERVICE_NAME" || true

echo "🚫 Zakazuji službu $SERVICE_NAME..."
sudo systemctl disable "$SERVICE_NAME" || true

echo "🗑️ Odstraňuji soubor služby $SERVICE_PATH..."
sudo rm -f "$SERVICE_PATH"

echo "🔄 Aktualizuji systemd..."
sudo systemctl daemon-reload
sudo systemctl reset-failed

echo "✅ Služba $SERVICE_NAME byla úspěšně odregistrována."
