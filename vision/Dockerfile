FROM ultralytics/ultralytics:latest-jetson-jetpack6

# Aktualizace a instalace systémových závislostí (např. zbar pro čtení QR kódů)
RUN apt-get update && apt-get install -y \
    libzbar0 \
    && rm -rf /var/lib/apt/lists/*

# Instalace nezbytných dodatečných Python knihoven (např. ZeroMQ)
RUN pip install --no-cache-dir pyzmq pyzbar

# Vygenerování YOLO konfigurace při sestavení, aby to nespamovalo logy při každém startu
#RUN yolo settings
