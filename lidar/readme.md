# Služba robot_lidar_tcp (LiDAR Unitree L2)

Tento program integruje tři služby běžící v oddělených vláknech nad společným bez-zámkovým kruhovým bufferem bodů:
- **Port 9002 (`LIDAR`)**: Nízkoúrovňový ovladač laseru, kalibrace s pevnou maskou podvozku (`x ∈ [-0.70, +0.23] m`, `y ∈ [-0.35, +0.35] m`), čtení UDP paketů, ZMQ PUB `DISTANCE` na `ipc:///tmp/robot-lidar`.
- **Port 9222 (`LIDAR-COLLISIONS`)**: Vyhodnocování kolizních kurzů v podadresáři [`collisions/`](file:///e:/robotour/robot-platform/lidar/collisions/kolizni_kurzy.md) (72 sektorů po 5° CW, CBDR kolizní predikce, ZMQ PUB `COLLISIONS` na `ipc:///tmp/robot-lidar-collisions`).
- **Port 9223 (`LIDAR-TRAVERSABILITY`)**: Lokální potenciálová mapa sjízdnosti terénu v podadresáři [`traversability/`](file:///e:/robotour/robot-platform/lidar/traversability/traversability.md) (`80 × 80` buněk po `10 cm`, stabilizovaná North-Up paměť `140 × 140`, ZMQ PUB `TRAVERSABILITY` na `ipc:///tmp/robot-lidar-traversability`).

## Build sekvence
```bash
mkdir -p build && cd build
cmake ..
make -j$(nproc)
chmod +x ../bin/robot_lidar_tcp
sudo systemctl restart robot-lidar
```

# set mode
Bit	Function                                              - Value 0                     Value 1
0	Switch between standard FOV and wide-angle FOV	      - Standard FOV (180°)         Wide-angle FOV (192°)
1	Switch between 3D and 2D measurement mode	          - 3D measurement mode         2D measurement mode
2	Enable or disable IMU                                 - Disable IMU                 Enable IMU            
3	Switch between network port mode and serial port mode - Network port mode           Serial port mode
4	Switch the default startup mode of the laser radar    - AutoStart when powered on   Keep it from rotating and wait for the startup command when powered on
5-31	Reserved	Reserved	Reserved
