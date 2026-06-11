#!/bin/bash

echo "🛑 Zastavuji a odstraňuji robot-vision.service..."
sudo systemctl stop robot-vision.service
sudo systemctl disable robot-vision.service
sudo rm -f /etc/systemd/system/robot-vision.service

echo "🧹 Čistím zbytkové docker kontejnery (pokud nějaké zbyly)..."
sudo bash -c 'docker inspect robot-vision >/dev/null 2>&1 && docker rm -f robot-vision || true'

echo "✅ Odregistrování dokončeno."
