#!/bin/bash

# ==========================================
# 1. NASTAVENÍ PROMĚNNÝCH
# ==========================================
PROJECT_DIR="/opt/projects/robotour"
SERVICE_DIR="$PROJECT_DIR/vision"        # Složka konkrétní služby
CONFIG_DIR="$SERVICE_DIR/config"         # Konfigurace žije izolovaně se službou

SERVICE_FILE="/etc/systemd/system/robot-vision.service"
LOG_DIR="/data/logs/vision"
LOG_FILE="$LOG_DIR/vision.log"

# ==========================================
# 2. PŘÍPRAVA ADRESÁŘŮ A PRÁV
# ==========================================
echo "📁 Vytvářím logovací složku ($LOG_DIR)..."
sudo mkdir -p "$LOG_DIR"
sudo touch "$LOG_FILE"
sudo chmod 664 "$LOG_FILE"

echo "📁 Vytvářím složku pro trvalou konfiguraci YOLO ($CONFIG_DIR)..."
sudo mkdir -p "$CONFIG_DIR"
# Důležité: Předáme práva uživateli 1000, aby do ní mohl Docker zapisovat
sudo chown -R 1000:1000 "$CONFIG_DIR"

# ==========================================
# 3. SESTAVENÍ DOCKER OBRAZU
# ==========================================
echo "🐳 Sestavuji lokální Docker obraz (robotour-vision)..."
# Ujistěte se, že v /opt/projects/robotour/vision/Dockerfile už NEMÁTE řádek 'RUN yolo settings'
sudo docker build -t robotour-vision "$SERVICE_DIR"

# ==========================================
# 4. INICIALIZACE YOLO KONFIGURACE
# ==========================================
echo "⚙️ Generuji trvalou YOLO konfiguraci na hostitelský disk..."
# Propíchneme se do složky vision a donutíme YOLO vygenerovat nastavení a stáhnout fonty
sudo docker run --rm \
  --user 1000:1000 \
  -v "$SERVICE_DIR:$SERVICE_DIR" \
  -e YOLO_CONFIG_DIR="$CONFIG_DIR" \
  robotour-vision yolo settings

# ==========================================
# 5. VYTVOŘENÍ SYSTEMD SLUŽBY
# ==========================================
echo "🛠️ Vytvářím systemd službu: robot-vision.service"

sudo tee "$SERVICE_FILE" > /dev/null <<EOF
[Unit]
Description=Robotour 2025 – vision socket server (Docker)
After=network.target docker.service
Requires=docker.service

[Service]
User=user
WorkingDirectory=$SERVICE_DIR

# Vlastní spuštění (un-buffer mód kvůli okamžitému logování v Pythonu)
Environment=PYTHONUNBUFFERED=1

# Před spuštěním ukonči libovolný proces, který drží port 9011
ExecStartPre=/bin/bash -c '/usr/bin/fuser -k 9011/tcp || true'
# Ukonči případný zamrzlý docker kontejner (potichu, pokud neexistuje)
ExecStartPre=/bin/bash -c 'docker inspect robot-vision >/dev/null 2>&1 && docker rm -f robot-vision || true'
# Smaž případný starý ZMQ socket, pokud ho vlastnil root
ExecStartPre=+/bin/rm -f /tmp/robot-vision
ExecStartPre=/bin/sleep 0.5

# Ostrý start kontejneru
ExecStart=/usr/bin/docker run --name robot-vision --rm \\
  --log-driver=none \\
  --user 1000:1000 \\
  -v /etc/passwd:/etc/passwd:ro \\
  -v /etc/group:/etc/group:ro \\
  -e HOME=/tmp \\
  -e YOLO_CONFIG_DIR=$CONFIG_DIR \\
  --ipc=host \\
  --net=host \\
  --runtime nvidia \\
  -v /tmp:/tmp \\
  -v /data:/data \\
  -v $SERVICE_DIR:$SERVICE_DIR \\
  -w $SERVICE_DIR \\
  robotour-vision python3 main.py

# Systemd čisté ukončení - zastaví docker kontejner s 5s timeoutem
ExecStop=/usr/bin/docker stop -t 5 robot-vision

# Logujeme přes systemd přesměrování (zabrání duplikaci díky --log-driver=none výše)
StandardOutput=append:$LOG_FILE
StandardError=append:$LOG_FILE

Restart=always
RestartSec=2

[Install]
WantedBy=multi-user.target
EOF

# ==========================================
# 6. AKTIVACE A START
# ==========================================
echo "🔄 Aktualizuji systemd a aktivuji službu..."
sudo systemctl daemon-reexec
sudo systemctl daemon-reload
sudo systemctl enable --now robot-vision.service

echo "✅ Služba robot-vision úspěšně nasazena a spuštěna!"
echo "   Pro sledování logu zadejte: tail -f $LOG_FILE"