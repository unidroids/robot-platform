# Vision Služba - Práce s Dockerem

Služba `vision` běží kvůli knihovnám TensorRT a YOLOv8 izolovaně uvnitř kontejneru a sestavuje si vlastní lokální obraz s názvem `robotour-vision`. 

Tento lokální obraz vychází z oficiálního `ultralytics/ultralytics:latest-jetson-jetpack6` a přidává naše specifické knihovny (např. ZeroMQ a pyzbar).

## Sestavení kontejneru (Offline příprava na dílně)
Pokud přidáte do `Dockerfile` další Python balíčky nebo potřebujete image vytvořit poprvé (dělá to automaticky i instalační skript), zavolejte v této složce:
```bash
docker build -t robotour-vision /opt/projects/robotour/vision
```

## Spuštění kontejneru pro ladění
Pokud si chcete spustit kontejner interaktivně a hrát si s kódem v bashi přímo uvnitř:
```bash
docker run -it --ipc=host --net=host --runtime nvidia -v /tmp:/tmp -v /data:/data -v /opt/projects/robotour:/opt/projects/robotour -w /opt/projects/robotour/vision robotour-vision bash
```
