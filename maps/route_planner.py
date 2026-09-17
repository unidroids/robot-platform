# maps/route_planner.py
"""
Plánovač tras pro mapovou službu:
- 5metrová validační zóna: ověření, zda jsou start a cíl do 5m od komunikace.
- Napojení startu a cíle na mapovou síť (napojovací úseky).
- Vyhledání nejkratší trasy pomocí Dijkstrova algoritmu.
- Výpočet pravostranného posunu (offset vpravo) podle šířky komunikace:
    - šířka < 2.0 m: offset = 0.0 m (střed cesty)
    - 2.0 až 3.0 m: offset = 0.5 m
    - 3.0 až 4.0 m: offset = 0.75 m
    - šířka >= 4.0 m: offset = 1.0 m
- Formátování výstupu do JSON struktury kompatibilní s mapovým formátem (metadata, nodes, edges).
"""
from __future__ import annotations
import math
import uuid
from typing import Dict, List, Optional, Tuple, Any

import networkx as nx

try:
    from .geo_geometry import (
        calc_distance_and_azimuth,
        wgs84_to_enu,
        enu_to_wgs84,
        calc_right_offset_for_width,
        compute_segment_right_normal
    )
    from .map_graph import MapGraph, NearestMapPoint
except (ImportError, ValueError):
    from geo_geometry import (
        calc_distance_and_azimuth,
        wgs84_to_enu,
        enu_to_wgs84,
        calc_right_offset_for_width,
        compute_segment_right_normal
    )
    from map_graph import MapGraph, NearestMapPoint


class RoutePlanner:
    """
    Plánovač trasy po mapovém grafu s napojením z libovolných souřadnic a pravostranným offsetem.
    """

    MAX_CONNECT_DISTANCE_M = 5.0  # Maximální povolená vzdálenost pro napojení

    def __init__(self, map_graph: MapGraph):
        self.map_graph = map_graph

    def plan_route(
        self,
        start_lat: float,
        start_lon: float,
        goal_lat: float,
        goal_lon: float,
        service_start_time: str = ""
    ) -> Dict[str, Any]:
        """
        Hlavní metoda vyhledání trasy:
        Vrátí slovník s metadaty, uzly a hranami.
        """
        if not self.map_graph.is_loaded:
            return self._build_error_response(
                start_lat, start_lon, goal_lat, goal_lon,
                service_start_time,
                "Mapa není načtena v paměti."
            )

        ref_lat = self.map_graph.ref_lat
        ref_lon = self.map_graph.ref_lon

        # 1. Nalezení nejbližšího bodu na mapě pro start i cíl
        start_near = self.map_graph.find_nearest_point_on_map(start_lat, start_lon)
        goal_near = self.map_graph.find_nearest_point_on_map(goal_lat, goal_lon)

        # 2. Kontrola 5m limitu
        reasons: List[str] = []
        if start_near.distance_m > self.MAX_CONNECT_DISTANCE_M:
            reasons.append(
                f"Start je dále než 5m od nejbližšího místa na mapě. "
                f"Nejbližší místo je {start_near.distance_m:.2f} metrů s azimutem {start_near.azimuth_deg:.1f}° daleko."
            )
        if goal_near.distance_m > self.MAX_CONNECT_DISTANCE_M:
            reasons.append(
                f"Cílové souřadnice jsou dále než 5m od nejbližšího místa na mapě. "
                f"Nejbližší místo je {goal_near.distance_m:.2f} metrů s azimutem {goal_near.azimuth_deg:.1f}° daleko."
            )

        if reasons:
            return self._build_error_response(
                start_lat, start_lon, goal_lat, goal_lon,
                service_start_time,
                " ".join(reasons),
                start_dist=start_near.distance_m,
                goal_dist=goal_near.distance_m
            )

        # 3. Sestavení dočasného grafu s napojením startu a cíle
        g = self.map_graph.graph.copy()
        start_node_id = "route_start"
        goal_node_id = "route_goal"

        sx, sy = wgs84_to_enu(start_lat, start_lon, ref_lat, ref_lon)
        gx, gy = wgs84_to_enu(goal_lat, goal_lon, ref_lat, ref_lon)

        g.add_node(start_node_id, lat=start_lat, lon=start_lon, x=sx, y=sy, label="Start")
        g.add_node(goal_node_id, lat=goal_lat, lon=goal_lon, x=gx, y=gy, label="Cíl")

        # Připojení startu do grafu
        start_conn_node_id = self._attach_point_to_graph(
            g, start_node_id, start_near, ref_lat, ref_lon, "start_conn"
        )

        # Připojení cíle do grafu
        goal_conn_node_id = self._attach_point_to_graph(
            g, goal_node_id, goal_near, ref_lat, ref_lon, "goal_conn"
        )

        # 4. Výpočet nejkratší cesty (Dijkstra)
        try:
            path_node_ids = nx.shortest_path(
                g,
                source=start_node_id,
                target=goal_node_id,
                weight="length_m"
            )
        except nx.NetworkXNoPath:
            return self._build_error_response(
                start_lat, start_lon, goal_lat, goal_lon,
                service_start_time,
                "Mezi startem a cílem neexistuje průchozí cesta v mapovém grafu.",
                start_dist=start_near.distance_m,
                goal_dist=goal_near.distance_m
            )

        # 5. Extrakce posloupnosti bodů a šířek jednotlivých úseků
        raw_points_enu: List[Tuple[float, float]] = []
        raw_widths: List[float] = []
        edge_names: List[str] = []

        for nid in path_node_ids:
            nd = g.nodes[nid]
            raw_points_enu.append((nd["x"], nd["y"]))

        for i in range(len(path_node_ids) - 1):
            u = path_node_ids[i]
            v = path_node_ids[i + 1]
            edge_data = g.get_edge_data(u, v, {})
            w = edge_data.get("width_m", 4.0)
            raw_widths.append(w)
            edge_names.append(edge_data.get("name", f"Úsek {i+1}"))

        # 6. Přepočet na pravý jízdní pruh (Right-Side Offset)
        # Body trasy:
        # P0 = přesná pozice startu (robot tam stojí, nepřesouvá se)
        # P_end = přesná pozice cíle (robot tam končí)
        # Vnitřní body komunikace se posunou vpravo od středové čáry.
        shifted_points_enu = self._apply_right_offset(raw_points_enu, raw_widths)

        # 7. Sestavení výsledné mapové struktury (metadata, nodes, edges)
        res_nodes = []
        res_edges = []
        total_route_length = 0.0

        for idx, (px, py) in enumerate(shifted_points_enu):
            nid = path_node_ids[idx]
            lat, lon = enu_to_wgs84(px, py, ref_lat, ref_lon)
            label = g.nodes[nid].get("label", f"Bod {idx+1}")
            if idx == 0:
                label = "Start trasy"
            elif idx == len(shifted_points_enu) - 1:
                label = "Cíl trasy"

            res_nodes.append({
                "id": f"route_node_{idx+1}_{nid}",
                "lat": lat,
                "lon": lon,
                "label": label
            })

        for i in range(len(res_nodes) - 1):
            n1 = res_nodes[i]
            n2 = res_nodes[i + 1]
            p1_enu = shifted_points_enu[i]
            p2_enu = shifted_points_enu[i + 1]
            seg_len = math.hypot(p2_enu[0] - p1_enu[0], p2_enu[1] - p1_enu[1])
            total_route_length += seg_len

            w = raw_widths[i]
            off = calc_right_offset_for_width(w)

            name = edge_names[i]
            if i == 0 and path_node_ids[0] == start_node_id:
                name = "Přístup na trasu"
            elif i == len(res_nodes) - 2 and path_node_ids[-1] == goal_node_id:
                name = "Příjezd do cíle"

            res_edges.append({
                "id": f"route_edge_{i+1}",
                "from": n1["id"],
                "to": n2["id"],
                "name": name,
                "width_m": w,
                "offset_m": off,
                "length_m": round(seg_len, 2),
                "controlPoint": None,
                "coords": []
            })

        area_name = self.map_graph.metadata.get("area_name", "Robotour Map")

        return {
            "metadata": {
                "area_name": area_name,
                "type": "route_result",
                "service_start_time": service_start_time,
                "search_result": "found",
                "route_length_m": round(total_route_length, 2),
                "start_query": {"lat": start_lat, "lon": start_lon},
                "goal_query": {"lat": goal_lat, "lon": goal_lon},
                "start_distance_to_map_m": round(start_near.distance_m, 2),
                "goal_distance_to_map_m": round(goal_near.distance_m, 2),
                "reason": ""
            },
            "nodes": res_nodes,
            "edges": res_edges
        }

    def _attach_point_to_graph(
        self,
        g: nx.Graph,
        query_node_id: str,
        near_info: NearestMapPoint,
        ref_lat: float,
        ref_lon: float,
        prefix: str
    ) -> str:
        """
        Připojí dotazovaný uzel (start/cíl) do grafu k nejbližšímu místu.
        Pokud je nejbližší místo na úsečce, rozdělí ji novým vloženým uzlem.
        """
        if near_info.point_type == "node" and near_info.node_id:
            conn_node_id = near_info.node_id
            g.add_edge(
                query_node_id,
                conn_node_id,
                name="Napojení na komunikaci",
                width_m=near_info.road_width_m,
                length_m=near_info.distance_m
            )
            return conn_node_id

        # Bod leží na úsečce mezi edge_from a edge_to
        u = near_info.edge_from
        v = near_info.edge_to
        if not u or not v or not g.has_edge(u, v):
            # Záložní napojení na nejbližší uzel
            target = u if u and g.has_node(u) else list(g.nodes())[0]
            g.add_edge(
                query_node_id, target,
                name="Napojení na komunikaci",
                width_m=near_info.road_width_m,
                length_m=near_info.distance_m
            )
            return target

        edge_data = g.get_edge_data(u, v)
        orig_len = edge_data.get("length_m", 10.0)
        orig_width = edge_data.get("width_m", near_info.road_width_m)
        orig_name = edge_data.get("name", "Komunikace")

        t = max(0.01, min(0.99, near_info.t_param))

        # Vytvoření dočasného bodu rozdělení
        conn_node_id = f"{prefix}_split_{uuid.uuid4().hex[:6]}"
        split_x, split_y = wgs84_to_enu(near_info.lat, near_info.lon, ref_lat, ref_lon)
        g.add_node(
            conn_node_id,
            lat=near_info.lat,
            lon=near_info.lon,
            x=split_x,
            y=split_y,
            label="Bod napojení na komunikaci"
        )

        # Odstraníme původní hranu a nahradíme dvěma úseky
        g.remove_edge(u, v)

        g.add_edge(
            u, conn_node_id,
            name=orig_name,
            width_m=orig_width,
            length_m=orig_len * t
        )
        g.add_edge(
            conn_node_id, v,
            name=orig_name,
            width_m=orig_width,
            length_m=orig_len * (1.0 - t)
        )

        # Propojení query bodu s bodem napojení
        g.add_edge(
            query_node_id, conn_node_id,
            name="Napojení na komunikaci",
            width_m=orig_width,
            length_m=near_info.distance_m
        )

        return conn_node_id

    def _apply_right_offset(
        self,
        points: List[Tuple[float, float]],
        widths: List[float]
    ) -> List[Tuple[float, float]]:
        """
        Aplikuje pravostranný offset na trasu tak, aby:
        - První bod P0 zůstal přesně v místě startu robota.
        - Poslední bod P_end zůstal přesně v místě cíle.
        - Všechny body podél cesty byly posunuty vpravo o odpovídající offset.
        """
        n = len(points)
        if n <= 2:
            return list(points)

        # Spočteme normálové vektory a offsety pro každý úsek
        seg_normals: List[Tuple[float, float]] = []
        seg_offsets: List[float] = []

        for i in range(n - 1):
            ax, ay = points[i]
            bx, by = points[i + 1]
            nx_val, ny_val = compute_segment_right_normal(ax, ay, bx, by)
            seg_normals.append((nx_val, ny_val))
            w = widths[i] if i < len(widths) else 4.0
            seg_offsets.append(calc_right_offset_for_width(w))

        shifted: List[Tuple[float, float]] = []

        # Bod 0: Start zůstává beze změny
        shifted.append(points[0])

        # Vnitřní body komunikace (1 až n-2):
        for i in range(1, n - 1):
            n1x, n1y = seg_normals[i - 1]
            n2x, n2y = seg_normals[i]
            off1 = seg_offsets[i - 1]
            off2 = seg_offsets[i]
            off_avg = (off1 + off2) / 2.0

            sum_nx = n1x + n2x
            sum_ny = n1y + n2y
            sum_len = math.hypot(sum_nx, sum_ny)

            if sum_len < 1e-4:
                nx_m = n1x
                ny_m = n1y
                scale = 1.0
            else:
                nx_m = sum_nx / sum_len
                ny_m = sum_ny / sum_len
                cos_half = n1x * nx_m + n1y * ny_m
                scale = min(1.5, 1.0 / cos_half) if cos_half > 0.1 else 1.0

            px = points[i][0] + nx_m * off_avg * scale
            py = points[i][1] + ny_m * off_avg * scale
            shifted.append((px, py))

        # Poslední bod: Cíl zůstává beze změny
        shifted.append(points[-1])

        return shifted

    def _build_error_response(
        self,
        start_lat: float,
        start_lon: float,
        goal_lat: float,
        goal_lon: float,
        service_start_time: str,
        reason: str,
        start_dist: float = 0.0,
        goal_dist: float = 0.0
    ) -> Dict[str, Any]:
        """Sestaví chybovou odpověď při nenalezení trasy nebo překročení 5m limitu."""
        area_name = self.map_graph.metadata.get("area_name", "Robotour Map") if self.map_graph else ""
        return {
            "metadata": {
                "area_name": area_name,
                "type": "route_result",
                "service_start_time": service_start_time,
                "search_result": "cesta nenalezena",
                "route_length_m": 0.0,
                "start_query": {"lat": start_lat, "lon": start_lon},
                "goal_query": {"lat": goal_lat, "lon": goal_lon},
                "start_distance_to_map_m": round(start_dist, 2),
                "goal_distance_to_map_m": round(goal_dist, 2),
                "reason": reason
            },
            "nodes": [],
            "edges": []
        }
