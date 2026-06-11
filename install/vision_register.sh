#!/bin/bash

SERVICE_FILE="/etc/systemd/system/robot-vision.service"
LOG_DIR="/data/logs/vision"
LOG_FILE="$LOG_DIR/vision.log"

echo "📁 Vytvářím logovací složku..."
sudo mkdir -p "$LOG_DIR"
sudo touch "$LOG_FILE"
sudo chmod 664 "$LOG_FILE"

echo "🐳 Sestavuji lokální Docker obraz (robotour-vision)..."
sudo docker build -t robotour-vision /opt/projects/robotour/vision

echo "🛠️ Vytvářím systemd službu: robot-vision.service"

sudo tee "$SERVICE_FILE" > /dev/null <<EOF
[Unit]
Description=Robotour 2025 – vision socket server (Docker)
After=network.target docker.service
Requires=docker.service

[Service]
User=user
WorkingDirectory=/opt/projects/robotour/vision

# vlastní spuštění (un-buffer mód kvůli okamžitému logování)
Environment=PYTHONUNBUFFERED=1

# před spuštěním ukonči libovolný proces, který drží port 9011
ExecStartPre=/bin/bash -c '/usr/bin/fuser -k 9011/tcp || true'
# Ukonči případný zamrzlý docker kontejner (potichu, pokud neexistuje)
ExecStartPre=/bin/bash -c 'docker inspect robot-vision >/dev/null 2>&1 && docker rm -f robot-vision || true'
ExecStartPre=/bin/sleep 0.5

ExecStart=/usr/bin/docker run --name robot-vision --rm --ipc=host --net=host --runtime nvidia -v /tmp:/tmp -v /data:/data -v /opt/projects/robotour:/opt/projects/robotour -w /opt/projects/robotour/vision robotour-vision python3 main.py

# Systemd čisté ukončení - zastaví docker kontejner s 5s timeoutem
ExecStop=/usr/bin/docker stop -t 5 robot-vision

# logujeme přes systemd přesměrování
StandardOutput=append:/data/logs/vision/vision.log
StandardError=append:/data/logs/vision/vision.log

Restart=always
RestartSec=2

[Install]
WantedBy=multi-user.target
EOF

echo "🔁 Aktivuji službu robot-vision.service"
sudo systemctl daemon-reexec
sudo systemctl daemon-reload
sudo systemctl enable --now robot-vision.service
echo "   tail -f $LOG_FILE"
