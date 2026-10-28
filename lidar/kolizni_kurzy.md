# Kolizní kurzy a prostorové vyhodnocení překážek (LiDAR L2)

Tento dokument specifikuje návrh a implementaci služby kolizních kurzů **`LIDAR-KOLIZE`** (TCP port 9222). Služba běží v rámci jednoho společného procesu s nízkoúrovňovým ovladačem **`LIDAR`** (TCP port 9002), ale disponuje vlastní sadou vláken a plně odděleným řízením, diagnostikou i publikační sběrnicí.

Podrobná specifikace je uvedena v [`kolize/kolizni_kurzy.md`](file:///e:/robotour/robot-platform/lidar/kolize/kolizni_kurzy.md) a [`lidar-collisions/readme.md`](file:///e:/robotour/robot-platform/lidar-collisions/readme.md).
