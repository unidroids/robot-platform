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

    MAX_START_DISTANCE_M = 3.0  # Maximální povolená vzdálenost pro napojení startu
    MAX_GOAL_DISTANCE_M = 10.0  # Maximální povolená vzdálenost pro napojení cíle

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

        # 2. Kontrola limitů vzdálenosti od mapy (start max 3m, cíl max 10m)
        reasons: List[str] = []
        if start_near.distance_m > self.MAX_START_DISTANCE_M:
            reasons.append(
                f"Start je dále než 3m od nejbližšího místa na mapě. "
                f"Nejbližší místo je {start_near.distance_m:.2f} metrů s azimutem {start_near.azimuth_deg:.1f}° daleko."
            )
        if goal_near.distance_m > self.MAX_GOAL_DISTANCE_M:
            reasons.append(
                f"Cílové souřadnice jsou dále než 10m od nejbližšího místa na mapě. "
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

        # 3. Sestavení dočasného grafu s napojením startu a cíle na komunikaci
        g = self.map_graph.graph.copy()
        start_node_id = "route_start"

        sx, sy = wgs84_to_enu(start_lat, start_lon, ref_lat, ref_lon)
        g.add_node(start_node_id, lat=start_lat, lon=start_lon, x=sx, y=sy, label="Start trasy")

        # Napojení startu robota na mapovou síť a nalezení cílového bodu na komunikaci
        # Pozn.: Cílový uzel končí přímo na komunikaci (nevytváří se odbočka k off-road souřadnicím)
        start_conn_node_id, goal_target_node_id = self._attach_start_and_find_goal(
            g,
            start_node_id,
            start_near,
            goal_near,
            ref_lat,
            ref_lon
        )

        # 4. Výpočet nejkratší cesty (Dijkstra)
        try:
            path_node_ids = nx.shortest_path(
                g,
                source=start_node_id,
                target=goal_target_node_id,
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
        # Všechny body podél cesty a cíl na cestě se posunou vpravo podle šířky komunikace.
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

    def _attach_start_and_find_goal(
        self,
        g: nx.Graph,
        start_node_id: str,
        start_near: NearestMapPoint,
        goal_near: NearestMapPoint,
        ref_lat: float,
        ref_lon: float
    ) -> Tuple[str, str]:
        """
        Napojí startovní pozici robota na mapovou síť (s vytvořením přístupové hrany)
        a připraví cílový uzel na komunikaci (bez vytváření koncové odbočky do cílových GPS).
        Robot tak zůstane stát na definované cestě v místě nejbližším cíli.
        """
        # Kontrola, zda start i cíl leží na téže hraně komunikace
        same_edge = (
            start_near.point_type == "edge" and
            goal_near.point_type == "edge" and
            start_near.edge_id is not None and
            start_near.edge_id == goal_near.edge_id and
            start_near.edge_from is not None and
            start_near.edge_to is not None and
            g.has_edge(start_near.edge_from, start_near.edge_to)
        )

        if same_edge:
            u = start_near.edge_from
            v = start_near.edge_to
            edge_data = g.get_edge_data(u, v)
            orig_len = edge_data.get("length_m", 10.0)
            orig_width = edge_data.get("width_m", start_near.road_width_m)
            orig_name = edge_data.get("name", "Komunikace")
            orig_id = edge_data.get("id", "e")

            ts = start_near.t_param
            tg = goal_near.t_param

            if abs(ts - tg) < 1e-4:
                split_id = f"conn_split_{uuid.uuid4().hex[:6]}"
                sx, sy = wgs84_to_enu(start_near.lat, start_near.lon, ref_lat, ref_lon)
                g.add_node(split_id, lat=start_near.lat, lon=start_near.lon, x=sx, y=sy, label="Bod napojení na komunikaci")
                g.remove_edge(u, v)
                t_clamped = max(0.005, min(0.995, ts))
                g.add_edge(u, split_id, id=f"{orig_id}_a", name=orig_name, width_m=orig_width, length_m=orig_len * t_clamped)
                g.add_edge(split_id, v, id=f"{orig_id}_b", name=orig_name, width_m=orig_width, length_m=orig_len * (1.0 - t_clamped))
                start_conn_node_id = split_id
                goal_target_node_id = split_id
            else:
                t1 = min(ts, tg)
                t2 = max(ts, tg)
                t1 = max(0.005, min(0.99, t1))
                t2 = max(t1 + 0.005, min(0.995, t2))

                lat1, lon1 = (start_near.lat, start_near.lon) if ts < tg else (goal_near.lat, goal_near.lon)
                lat2, lon2 = (goal_near.lat, goal_near.lon) if ts < tg else (start_near.lat, start_near.lon)

                split1_id = f"conn_split1_{uuid.uuid4().hex[:6]}"
                split2_id = f"conn_split2_{uuid.uuid4().hex[:6]}"
                x1, y1 = wgs84_to_enu(lat1, lon1, ref_lat, ref_lon)
                x2, y2 = wgs84_to_enu(lat2, lon2, ref_lat, ref_lon)

                g.add_node(split1_id, lat=lat1, lon=lon1, x=x1, y=y1, label="Bod na komunikaci")
                g.add_node(split2_id, lat=lat2, lon=lon2, x=x2, y=y2, label="Bod na komunikaci")

                g.remove_edge(u, v)
                g.add_edge(u, split1_id, id=f"{orig_id}_a", name=orig_name, width_m=orig_width, length_m=orig_len * t1)
                g.add_edge(split1_id, split2_id, id=f"{orig_id}_mid", name=orig_name, width_m=orig_width, length_m=orig_len * (t2 - t1))
                g.add_edge(split2_id, v, id=f"{orig_id}_b", name=orig_name, width_m=orig_width, length_m=orig_len * (1.0 - t2))

                if ts < tg:
                    start_conn_node_id = split1_id
                    goal_target_node_id = split2_id
                else:
                    start_conn_node_id = split2_id
                    goal_target_node_id = split1_id

            g.add_edge(
                start_node_id,
                start_conn_node_id,
                name="Přístup na trasu",
                width_m=start_near.road_width_m,
                length_m=start_near.distance_m
            )
            return start_conn_node_id, goal_target_node_id

        # Pokud neleží na téže hraně:
        # 1. Cílový bod na komunikaci (bez odbočky do off-road GPS)
        if goal_near.point_type == "node" and goal_near.node_id:
            goal_target_node_id = goal_near.node_id
        else:
            goal_target_node_id = self._split_edge_at_point(
                g, goal_near, ref_lat, ref_lon, prefix="goal_conn"
            )

        # 2. Napojení startu robota na komunikaci
        if start_near.point_type == "node" and start_near.node_id:
            start_conn_node_id = start_near.node_id
        else:
            start_conn_node_id = self._split_edge_at_point(
                g, start_near, ref_lat, ref_lon, prefix="start_conn"
            )

        g.add_edge(
            start_node_id,
            start_conn_node_id,
            name="Přístup na trasu",
            width_m=start_near.road_width_m,
            length_m=start_near.distance_m
        )

        return start_conn_node_id, goal_target_node_id

    def _split_edge_at_point(
        self,
        g: nx.Graph,
        near_info: NearestMapPoint,
        ref_lat: float,
        ref_lon: float,
        prefix: str
    ) -> str:
        """
        Vloží do grafu nový uzel na úsečku komunikace a rozdělí původní hranu na dvě.
        Vrátí ID nově vloženého uzlu na komunikaci.
        """
        u = near_info.edge_from
        v = near_info.edge_to
        if not u or not v or not g.has_edge(u, v):
            target = u if u and g.has_node(u) else list(g.nodes())[0]
            return target

        edge_data = g.get_edge_data(u, v)
        orig_len = edge_data.get("length_m", 10.0)
        orig_width = edge_data.get("width_m", near_info.road_width_m)
        orig_name = edge_data.get("name", "Komunikace")
        orig_id = edge_data.get("id", "e")

        t = max(0.005, min(0.995, near_info.t_param))

        split_node_id = f"{prefix}_split_{uuid.uuid4().hex[:6]}"
        split_x, split_y = wgs84_to_enu(near_info.lat, near_info.lon, ref_lat, ref_lon)
        g.add_node(
            split_node_id,
            lat=near_info.lat,
            lon=near_info.lon,
            x=split_x,
            y=split_y,
            label="Bod na komunikaci"
        )

        g.remove_edge(u, v)
        g.add_edge(
            u, split_node_id,
            id=f"{orig_id}_a",
            name=orig_name,
            width_m=orig_width,
            length_m=orig_len * t
        )
        g.add_edge(
            split_node_id, v,
            id=f"{orig_id}_b",
            name=orig_name,
            width_m=orig_width,
            length_m=orig_len * (1.0 - t)
        )

        return split_node_id

    def _apply_right_offset(
        self,
        points: List[Tuple[float, float]],
        widths: List[float]
    ) -> List[Tuple[float, float]]:
        """
        Aplikuje pravostranný offset na trasu tak, aby:
        - První bod P0 zůstal přesně v místě startu robota (aktuální poloha robota).
        - Všechny body podél cesty i cílový bod na cestě se posunuly vpravo podle šířky komunikace.
        """
        n = len(points)
        if n <= 1:
            return list(points)
        if n == 2:
            return list(points)

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

        # Bod 0: Start zůstává beze změny (skutečná poloha robota)
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

        # Poslední bod: Cíl na komunikaci - posunut vpravo podle posledního úseku komunikace
        p_last_x = points[-1][0] + seg_normals[-1][0] * seg_offsets[-1]
        p_last_y = points[-1][1] + seg_normals[-1][1] * seg_offsets[-1]
        shifted.append((p_last_x, p_last_y))

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
        """Sestaví chybovou odpověď při nenalezení trasy nebo překročení limitů vzdálenosti k mapě."""
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
