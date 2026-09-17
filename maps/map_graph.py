# maps/map_graph.py
"""
Správa mapového grafu:
- Načtení JSON mapového modelu (metadata, nodes, edges).
- Převod do vnitřní reprezentace NetworkX grafu s metrickými délkami hran.
- Geometrické vyhledávání nejbližšího místa na mapě (uzel či úsek) s přesnou vzdáleností a azimutem.
"""
from __future__ import annotations
import json
import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple, Any

import networkx as nx

try:
    from .geo_geometry import (
        calc_distance_and_azimuth,
        wgs84_to_enu,
        enu_to_wgs84,
        project_point_to_segment
    )
except (ImportError, ValueError):
    from geo_geometry import (
        calc_distance_and_azimuth,
        wgs84_to_enu,
        enu_to_wgs84,
        project_point_to_segment
    )


@dataclass
class MapNode:
    id: str
    lat: float
    lon: float
    label: str = ""
    x: float = 0.0  # Lokální ENU souřadnice (m)
    y: float = 0.0


@dataclass
class MapEdge:
    id: str
    from_node: str
    to_node: str
    name: str = ""
    width_m: float = 4.0
    length_m: float = 0.0
    control_point: Optional[Any] = None
    coords: List[Any] = field(default_factory=list)


@dataclass
class NearestMapPoint:
    """Informace o nejbližším bodu na mapě vzhledem k dotazované pozici."""
    lat: float
    lon: float
    distance_m: float
    azimuth_deg: float
    point_type: str            # 'node' nebo 'edge'
    node_id: Optional[str] = None
    edge_id: Optional[str] = None
    edge_from: Optional[str] = None
    edge_to: Optional[str] = None
    t_param: float = 0.0       # 0.0 = edge_from, 1.0 = edge_to
    road_width_m: float = 4.0


class MapGraph:
    """
    Třída reprezentující topologický graf cest a jeho geometrické vlastnosti.
    """

    def __init__(self):
        self.metadata: Dict[str, Any] = {}
        self.nodes: Dict[str, MapNode] = {}
        self.edges: List[MapEdge] = []
        self.graph = nx.Graph()
        self.ref_lat: float = 0.0
        self.ref_lon: float = 0.0
        self.is_loaded: bool = False

    def load_from_file(self, file_path: str) -> bool:
        """Načte mapový graf z JSON souboru."""
        with open(file_path, "r", encoding="utf-8") as f:
            data = json.load(f)
        return self.load_from_dict(data)

    def load_from_dict(self, data: Dict[str, Any]) -> bool:
        """Načte mapový graf ze slovníku."""
        self.metadata = data.get("metadata", {})
        nodes_raw = data.get("nodes", [])
        edges_raw = data.get("edges", [])

        if not nodes_raw:
            return False

        # Referenční bod pro lokální ENU souřadnice (např. první bod nebo průměr)
        lats = [float(n["lat"]) for n in nodes_raw]
        lons = [float(n["lon"]) for n in nodes_raw]
        self.ref_lat = sum(lats) / len(lats)
        self.ref_lon = sum(lons) / len(lons)

        self.nodes.clear()
        self.edges.clear()
        self.graph.clear()

        # 1. Načtení uzlů
        for n in nodes_raw:
            nid = str(n["id"])
            lat = float(n["lat"])
            lon = float(n["lon"])
            label = str(n.get("label", ""))
            x, y = wgs84_to_enu(lat, lon, self.ref_lat, self.ref_lon)
            map_node = MapNode(id=nid, lat=lat, lon=lon, label=label, x=x, y=y)
            self.nodes[nid] = map_node
            self.graph.add_node(nid, lat=lat, lon=lon, x=x, y=y, label=label)

        # 2. Načtení hran
        for e in edges_raw:
            eid = str(e.get("id", f"edge_{len(self.edges)}"))
            fn = str(e["from"])
            tn = str(e["to"])
            name = str(e.get("name", ""))
            width_m = float(e.get("width_m", 4.0))

            if fn not in self.nodes or tn not in self.nodes:
                continue

            node_from = self.nodes[fn]
            node_to = self.nodes[tn]

            length_m = math.hypot(node_to.x - node_from.x, node_to.y - node_from.y)

            map_edge = MapEdge(
                id=eid,
                from_node=fn,
                to_node=tn,
                name=name,
                width_m=width_m,
                length_m=length_m,
                control_point=e.get("controlPoint"),
                coords=e.get("coords", [])
            )
            self.edges.append(map_edge)

            # Do NetworkX grafu přidáváme neorientovanou hranu (cesty v parku jsou obousměrné)
            self.graph.add_edge(
                fn, tn,
                id=eid,
                name=name,
                width_m=width_m,
                length_m=length_m
            )

        self.is_loaded = True
        return True

    def find_nearest_point_on_map(self, lat: float, lon: float) -> NearestMapPoint:
        """
        Nalezne nejbližší bod na celé mapě (prohledá všechny uzly a úsečky hran).
        Vrátí NearestMapPoint s přesnou vzdáleností a azimutem ze zadané pozice.
        """
        if not self.is_loaded or not self.nodes:
            raise RuntimeError("Mapa není načtena.")

        qx_query, qy_query = wgs84_to_enu(lat, lon, self.ref_lat, self.ref_lon)

        best_dist = float("inf")
        best_x = 0.0
        best_y = 0.0
        best_type = "node"
        best_node_id: Optional[str] = None
        best_edge_id: Optional[str] = None
        best_from: Optional[str] = None
        best_to: Optional[str] = None
        best_t = 0.0
        best_width = 4.0

        # 1. Prohledáme všechny úsečky hran (nejpřesnější napojení na komunikaci)
        for e in self.edges:
            nf = self.nodes[e.from_node]
            nt = self.nodes[e.to_node]

            proj_x, proj_y, dist, t = project_point_to_segment(
                qx_query, qy_query,
                nf.x, nf.y,
                nt.x, nt.y
            )

            if dist < best_dist:
                best_dist = dist
                best_x = proj_x
                best_y = proj_y
                best_edge_id = e.id
                best_from = e.from_node
                best_to = e.to_node
                best_t = t
                best_width = e.width_m

                if t <= 1e-4:
                    best_type = "node"
                    best_node_id = e.from_node
                elif t >= 1.0 - 1e-4:
                    best_type = "node"
                    best_node_id = e.to_node
                else:
                    best_type = "edge"
                    best_node_id = None

        # 2. Zkontrolujeme i izolované uzly (pokud by existovaly)
        for nid, node in self.nodes.items():
            dist = math.hypot(qx_query - node.x, qy_query - node.y)
            if dist < best_dist:
                best_dist = dist
                best_x = node.x
                best_y = node.y
                best_type = "node"
                best_node_id = nid
                best_edge_id = None
                best_from = None
                best_to = None
                best_t = 0.0
                best_width = 4.0

        # Převod nalezeného bodu zpět na WGS84
        res_lat, res_lon = enu_to_wgs84(best_x, best_y, self.ref_lat, self.ref_lon)

        # Výpočet přesné elipsoidické vzdálenosti a azimutu z query bodu do nalezeného bodu
        exact_dist, azimuth = calc_distance_and_azimuth(lat, lon, res_lat, res_lon)

        return NearestMapPoint(
            lat=res_lat,
            lon=res_lon,
            distance_m=exact_dist,
            azimuth_deg=azimuth,
            point_type=best_type,
            node_id=best_node_id,
            edge_id=best_edge_id,
            edge_from=best_from,
            edge_to=best_to,
            t_param=best_t,
            road_width_m=best_width
        )
