# Kolizní kurzy a prostorové vyhodnocení překážek (LiDAR L2)

Implementace služby kolizních kurzů **`LIDAR-KOLIZE`** (TCP port 9222) běží v rámci jednoho společného procesu služby **`LIDAR`** (TCP port 9002). To umožňuje optimalizaci výkonu a sdílení dat mezi oběma službami.

