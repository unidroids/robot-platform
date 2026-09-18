# maps/test_maps.py
"""
Jednotkové a integrační testy mapové služby (MAPS):
- Geometrické funkce (WGS84 <-> ENU, projekce, pravostranné offsety).
- Načtení mapového grafu Kramolín (uzly, hrany).
- Hledání trasy, 5m validační pravidlo, pravostranný posun.
- TCP příkazy (PING, STATUS, FIND_ROUTE, EXIT).
"""
import json
import math
import os
import socket
import sys
import threading
import time
import unittest

# Zajištění importu modulu maps
BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if BASE_DIR not in sys.path:
    sys.path.insert(0, BASE_DIR)

from maps.geo_geometry import (
    calc_distance_and_azimuth,
    wgs84_to_enu,
    enu_to_wgs84,
    project_point_to_segment,
    calc_right_offset_for_width,
    compute_segment_right_normal,
    offset_polyline_right
)
from maps.map_graph import MapGraph
from maps.route_planner import RoutePlanner
from maps.service import MapService
from maps.client import client_thread, parse_find_route_args


class TestGeoGeometry(unittest.TestCase):
    """Testy čistých matematických a geodetických výpočtů."""

    def test_distance_and_azimuth(self):
        # 1. Směr na sever (lat1 < lat2, lon1 == lon2)
        dist_n, az_n = calc_distance_and_azimuth(50.0, 14.0, 50.001, 14.0)
        self.assertAlmostEqual(dist_n, 111.2, delta=0.5)
        self.assertAlmostEqual(az_n, 0.0, delta=0.1)

        # 2. Směr na východ (lat1 == lat2, lon1 < lon2)
        dist_e, az_e = calc_distance_and_azimuth(50.0, 14.0, 50.0, 14.001)
        self.assertAlmostEqual(dist_e, 71.5, delta=0.5)
        self.assertAlmostEqual(az_e, 90.0, delta=0.1)

        # 3. Směr na jih (azimut 180°)
        dist_s, az_s = calc_distance_and_azimuth(50.001, 14.0, 50.0, 14.0)
        self.assertAlmostEqual(dist_s, 111.2, delta=0.5)
        self.assertAlmostEqual(az_s, 180.0, delta=0.1)

    def test_enu_roundtrip(self):
        ref_lat, ref_lon = 49.554, 12.741
        p_lat, p_lon = 49.555, 12.742

        x, y = wgs84_to_enu(p_lat, p_lon, ref_lat, ref_lon)
        lat_back, lon_back = enu_to_wgs84(x, y, ref_lat, ref_lon)

        self.assertAlmostEqual(lat_back, p_lat, places=7)
        self.assertAlmostEqual(lon_back, p_lon, places=7)

    def test_projection_to_segment(self):
        # Úsečka z (0, 0) do (10, 0) podél osy X
        ax, ay = 0.0, 0.0
        bx, by = 10.0, 0.0

        # Bod nad středem úsečky (5, 3) -> průmět (5, 0), vzdálenost 3
        qx, qy, dist, t = project_point_to_segment(5.0, 3.0, ax, ay, bx, by)
        self.assertAlmostEqual(qx, 5.0)
        self.assertAlmostEqual(qy, 0.0)
        self.assertAlmostEqual(dist, 3.0)
        self.assertAlmostEqual(t, 0.5)

        # Bod před začátkem (-2, 4) -> průmět (0, 0), vzdálenost hypot(2, 4)
        qx, qy, dist, t = project_point_to_segment(-2.0, 4.0, ax, ay, bx, by)
        self.assertAlmostEqual(qx, 0.0)
        self.assertAlmostEqual(qy, 0.0)
        self.assertAlmostEqual(dist, math.hypot(2.0, 4.0))
        self.assertEqual(t, 0.0)

        # Bod za koncem (12, 0) -> průmět (10, 0), vzdálenost 2
        qx, qy, dist, t = project_point_to_segment(12.0, 0.0, ax, ay, bx, by)
        self.assertAlmostEqual(qx, 10.0)
        self.assertAlmostEqual(qy, 0.0)
        self.assertAlmostEqual(dist, 2.0)
        self.assertEqual(t, 1.0)

    def test_right_offset_stepped_rule(self):
        # < 2m -> 0.0m
        self.assertEqual(calc_right_offset_for_width(1.5), 0.0)
        self.assertEqual(calc_right_offset_for_width(1.99), 0.0)

        # 2.0 až 3.0m -> 0.5m
        self.assertEqual(calc_right_offset_for_width(2.0), 0.5)
        self.assertEqual(calc_right_offset_for_width(2.5), 0.5)
        self.assertEqual(calc_right_offset_for_width(2.99), 0.5)

        # 3.0 až 4.0m -> 0.75m
        self.assertEqual(calc_right_offset_for_width(3.0), 0.75)
        self.assertEqual(calc_right_offset_for_width(3.5), 0.75)
        self.assertEqual(calc_right_offset_for_width(3.99), 0.75)

        # >= 4.0m -> 1.0m
        self.assertEqual(calc_right_offset_for_width(4.0), 1.0)
        self.assertEqual(calc_right_offset_for_width(4.5), 1.0)
        self.assertEqual(calc_right_offset_for_width(6.0), 1.0)

    def test_polyline_offset_right(self):
        # Jízda na sever: (0, 0) -> (0, 20), šířka 4.5m -> posun vpravo (+X) o 1.0m
        poly = [(0.0, 0.0), (0.0, 20.0)]
        shifted = offset_polyline_right(poly, [4.5])
        self.assertAlmostEqual(shifted[0][0], 1.0)
        self.assertAlmostEqual(shifted[0][1], 0.0)
        self.assertAlmostEqual(shifted[1][0], 1.0)
        self.assertAlmostEqual(shifted[1][1], 20.0)


class TestMapGraph(unittest.TestCase):
    """Testy načítání mapy a prostorového vyhledávání."""

    @classmethod
    def setUpClass(cls):
        cls.map_file = os.path.join(os.path.dirname(__file__), "defaut_map.json")
        cls.mg = MapGraph()
        loaded = cls.mg.load_from_file(cls.map_file)
        assert loaded, "Nepodařilo se načíst výchozí mapu defaut_map.json"

    def test_map_loaded(self):
        self.assertTrue(self.mg.is_loaded)
        self.assertEqual(len(self.mg.nodes), 43)
        self.assertEqual(len(self.mg.edges), 45)
        self.assertIn("Kramolín", self.mg.metadata.get("area_name", ""))

    def test_nearest_point_on_node(self):
        n1 = self.mg.nodes["osm_node_1"]
        near = self.mg.find_nearest_point_on_map(n1.lat, n1.lon)
        self.assertAlmostEqual(near.distance_m, 0.0, delta=0.01)
        self.assertEqual(near.point_type, "node")
        self.assertEqual(near.node_id, "osm_node_1")

    def test_nearest_point_on_edge(self):
        # Posun o 1m kolmo od uzlu 1
        n1 = self.mg.nodes["osm_node_1"]
        near = self.mg.find_nearest_point_on_map(n1.lat + 0.00001, n1.lon + 0.00001)
        self.assertLess(near.distance_m, 5.0)
        self.assertIn(near.point_type, ("edge", "node"))


class TestRoutePlanner(unittest.TestCase):
    """Testy plánování tras, 5m pravidla a formátu JSON."""

    @classmethod
    def setUpClass(cls):
        cls.map_file = os.path.join(os.path.dirname(__file__), "defaut_map.json")
        cls.mg = MapGraph()
        cls.mg.load_from_file(cls.map_file)
        cls.planner = RoutePlanner(cls.mg)

    def test_route_within_5m_success(self):
        # Start u uzlu 1, cíl u uzlu 14
        n1 = self.mg.nodes["osm_node_1"]
        n14 = self.mg.nodes["osm_node_14"]

        # Dotaz 1m vedle uzlu 1 a 1m vedle uzlu 14
        s_lat = n1.lat + 0.000005
        s_lon = n1.lon + 0.000005
        g_lat = n14.lat - 0.000005
        g_lon = n14.lon - 0.000005

        res = self.planner.plan_route(s_lat, s_lon, g_lat, g_lon, "2026-09-17T07:20:00Z")
        meta = res["metadata"]

        self.assertEqual(meta["search_result"], "found")
        self.assertGreater(meta["route_length_m"], 100.0)
        self.assertEqual(meta["reason"], "")
        self.assertGreaterEqual(len(res["nodes"]), 2)
        self.assertGreaterEqual(len(res["edges"]), 1)

        # První bod trasy musí odpovídat startu
        self.assertAlmostEqual(res["nodes"][0]["lat"], s_lat, places=6)
        self.assertAlmostEqual(res["nodes"][0]["lon"], s_lon, places=6)

        # Poslední bod trasy musí odpovídat cíli na komunikaci (žádná koncová odbočka)
        self.assertEqual(res["nodes"][-1]["label"], "Cíl trasy")
        last_node = res["nodes"][-1]
        last_dist = self.mg.find_nearest_point_on_map(last_node["lat"], last_node["lon"]).distance_m
        self.assertLessEqual(last_dist, 1.5)

        # Žádná hrana nesmí být odbočka "Příjezd do cíle"
        for e in res["edges"]:
            self.assertNotEqual(e["name"], "Příjezd do cíle")

        # Všechny hrany mají specifikovaný offset
        for e in res["edges"]:
            self.assertIn("offset_m", e)
            self.assertGreaterEqual(e["offset_m"], 0.0)

    def test_no_offroad_goal_branch(self):
        # Start u uzlu 1, cíl 3 metry vedle uzlu 14 v trávě
        n1 = self.mg.nodes["osm_node_1"]
        n14 = self.mg.nodes["osm_node_14"]

        g_lat = n14.lat
        g_lon = n14.lon + 0.00004  # cca 2.88 metru kolmo od komunikace

        res = self.planner.plan_route(n1.lat, n1.lon, g_lat, g_lon)
        meta = res["metadata"]

        self.assertEqual(meta["search_result"], "found")
        self.assertGreater(meta["goal_distance_to_map_m"], 2.5)
        self.assertLessEqual(meta["goal_distance_to_map_m"], 5.0)

        # Cílový bod trasy nesmí odpovídat zadaným GPS souřadnicím cíle v trávě
        last_node = res["nodes"][-1]
        self.assertNotAlmostEqual(last_node["lon"], g_lon, places=5)
        self.assertEqual(last_node["label"], "Cíl trasy")

        # Cílový bod musí ležet na komunikaci
        last_pt_dist = self.mg.find_nearest_point_on_map(last_node["lat"], last_node["lon"]).distance_m
        self.assertLessEqual(last_pt_dist, 1.5)

    def test_route_start_and_goal_on_same_edge(self):
        # Start i cíl leží podél stejné hrany (osm_node_5 -> osm_node_6)
        n5 = self.mg.nodes["osm_node_5"]
        n6 = self.mg.nodes["osm_node_6"]

        # Start v cca 20% úseku, cíl v cca 70% úseku
        s_lat = n5.lat * 0.8 + n6.lat * 0.2 + 0.00001
        s_lon = n5.lon * 0.8 + n6.lon * 0.2 + 0.00001
        g_lat = n5.lat * 0.3 + n6.lat * 0.7
        g_lon = n5.lon * 0.3 + n6.lon * 0.7

        res = self.planner.plan_route(s_lat, s_lon, g_lat, g_lon)
        self.assertEqual(res["metadata"]["search_result"], "found")
        self.assertEqual(len(res["nodes"]), 3)
        self.assertEqual(res["nodes"][0]["label"], "Start trasy")
        self.assertEqual(res["nodes"][1]["label"], "Bod na komunikaci")
        self.assertEqual(res["nodes"][2]["label"], "Cíl trasy")
        self.assertEqual(res["edges"][0]["name"], "Přístup na trasu")
        self.assertEqual(res["edges"][1]["name"], "OSM Úsek 4")
        self.assertGreater(res["metadata"]["route_length_m"], 5.0)

    def test_rejection_start_too_far(self):
        n1 = self.mg.nodes["osm_node_1"]
        n14 = self.mg.nodes["osm_node_14"]

        # Start 50 metrů daleko v poli
        s_lat = n1.lat + 0.0005
        s_lon = n1.lon

        res = self.planner.plan_route(s_lat, s_lon, n14.lat, n14.lon)
        meta = res["metadata"]

        self.assertEqual(meta["search_result"], "cesta nenalezena")
        self.assertIn("Start je dále než 5m od nejbližšího místa na mapě", meta["reason"])
        self.assertGreater(meta["start_distance_to_map_m"], 5.0)
        self.assertEqual(len(res["nodes"]), 0)
        self.assertEqual(len(res["edges"]), 0)

    def test_rejection_goal_too_far(self):
        n1 = self.mg.nodes["osm_node_1"]
        n14 = self.mg.nodes["osm_node_14"]

        # Cíl 80 metrů daleko
        g_lat = n14.lat + 0.0008
        g_lon = n14.lon

        res = self.planner.plan_route(n1.lat, n1.lon, g_lat, g_lon)
        meta = res["metadata"]

        self.assertEqual(meta["search_result"], "cesta nenalezena")
        self.assertIn("Cílové souřadnice jsou dále než 5m od nejbližšího místa na mapě", meta["reason"])
        self.assertGreater(meta["goal_distance_to_map_m"], 5.0)


class TestMapServiceAndTCP(unittest.TestCase):
    """Testy služby a TCP síťové komunikace."""

    @classmethod
    def setUpClass(cls):
        cls.service = MapService()
        cls.test_port = 19040

        # Spuštění testovacího TCP serveru
        cls.server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        cls.server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        cls.server_sock.bind(("127.0.0.1", cls.test_port))
        cls.server_sock.listen(5)

        cls.running = True

        def server_loop():
            while cls.running:
                try:
                    client_s, addr = cls.server_sock.accept()
                    threading.Thread(
                        target=client_thread,
                        args=(client_s, addr, cls.service),
                        daemon=True
                    ).start()
                except Exception:
                    break

        cls.thread = threading.Thread(target=server_loop, daemon=True)
        cls.thread.start()
        time.sleep(0.1)

    @classmethod
    def tearDownClass(cls):
        cls.running = False
        try:
            cls.server_sock.close()
        except Exception:
            pass

    def _send_cmd(self, cmd: str) -> str:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.connect(("127.0.0.1", self.test_port))
        s.sendall((cmd + "\n").encode("utf-8"))
        f = s.makefile("rwb", buffering=0)
        resp = f.readline().decode("utf-8").strip()
        s.close()
        return resp

    def test_tcp_ping(self):
        resp = self._send_cmd("PING")
        self.assertEqual(resp, "PONG MAPS")

    def test_tcp_status(self):
        resp = self._send_cmd("STATUS")
        self.assertTrue(resp.startswith("READY"))
        parts = resp.split(" ", 1)
        data = json.loads(parts[1])
        self.assertEqual(data["service"], "MAPS")
        self.assertEqual(data["mode"], "READY")
        self.assertEqual(data["nodes_count"], 43)
        self.assertEqual(data["edges_count"], 45)

    def test_tcp_find_route_comma_and_spaces(self):
        n1 = self.service.map_graph.nodes["osm_node_1"]
        n14 = self.service.map_graph.nodes["osm_node_14"]

        # Test čárkami oddělených argumentů
        cmd = f"FIND_ROUTE {n1.lat}, {n1.lon}, {n14.lat}, {n14.lon}"
        resp_line = self._send_cmd(cmd)
        data = json.loads(resp_line)
        self.assertEqual(data["metadata"]["search_result"], "found")
        self.assertGreater(data["metadata"]["route_length_m"], 100.0)

        # Test mezerami oddělených argumentů
        cmd_spaces = f"FIND_ROUTE {n1.lat} {n1.lon} {n14.lat} {n14.lon}"
        resp_line_spaces = self._send_cmd(cmd_spaces)
        data_spaces = json.loads(resp_line_spaces)
        self.assertEqual(data_spaces["metadata"]["search_result"], "found")

    def test_tcp_find_route_out_of_bounds(self):
        n1 = self.service.map_graph.nodes["osm_node_1"]
        cmd = f"FIND_ROUTE {n1.lat + 0.001}, {n1.lon}, {n1.lat}, {n1.lon}"
        resp_line = self._send_cmd(cmd)
        data = json.loads(resp_line)
        self.assertEqual(data["metadata"]["search_result"], "cesta nenalezena")
        self.assertIn("Start je dále než 5m", data["metadata"]["reason"])

    def test_parse_args_helper(self):
        self.assertEqual(
            parse_find_route_args("1.0, 2.0, 3.0, 4.0"),
            [1.0, 2.0, 3.0, 4.0]
        )
        self.assertEqual(
            parse_find_route_args("1.0 2.0 3.0 4.0"),
            [1.0, 2.0, 3.0, 4.0]
        )
        self.assertIsNone(parse_find_route_args("1.0 2.0"))


if __name__ == "__main__":
    unittest.main()
