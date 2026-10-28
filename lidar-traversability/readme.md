# Služba LIDAR-TRAVERSABILITY (Potenciálová mapa sjízdnosti terénu)

Služba `LIDAR-TRAVERSABILITY` běží jako nezávislé vlákno v rámci binárky `robot_lidar_tcp`. Z 3D mračna bodů LiDARu Unitree L2 a plné orientace a polohy robota ze služby `fusion` (`ipc:///tmp/robot-fusion`, topic `SOLUTION`) sestavuje na frekvenci **10 Hz** lokální 2D potenciálovou mapu sjízdnosti (`80 × 80` buněk po `10 × 10 cm` = `8.0 × 8.0 m`).
