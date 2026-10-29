# pilot_maraton/corridor_planner.py
"""
Koridorový plánovač (Elastic Band / DP v koridoru [-2.0 m vlevo, +1.0 m vpravo])
a stavový automat pro vyhodnocování 3D překážek, zametené oblasti (swept zone),
předjíždění chodců jdoucích stejným směrem a detekci neprůjezdné cesty.
"""
from __future__ import annotations

import bisect
import math
import time
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional, Tuple

try:
    from .geo_utils import deg2rad, heading_enu_to_gnss
except (ImportError, ValueError):
    from geo_utils import deg2rad, heading_enu_to_gnss


# Rozměry koridoru pro střed otáčení robota (CoR) vůči ideální trase z MAPS:
# d_lat < 0 => VLEVO od ideální trasy, d_lat > 0 => VPRAVO od ideální trasy (CW konvence)
CORRIDOR_LEFT_M = 2.0
CORRIDOR_RIGHT_M = 1.0
LATERAL_STEP_M = 0.10
SLICE_STEP_M = 0.25
LOOKAHEAD_HORIZON_M = 5.50

# Parametry zastavení a čekání u 3D překážky
WAIT_STOP_DIST_M = 1.45          # Cílový odstup zastavení před 3D překážkou (~1.45 m)
APPROACH_SLOW_DIST_M = 4.50      # Začátek plynulého brzdění k překážce
EMERGENCY_STOP_DIST_M = 0.70     # Tvrdá nouzová hranice (0.7 m)

BASE_WAIT_SEC = 5.0              # Základní čekání u překážky (5 s)
EXTEND_WAIT_STEP_SEC = 5.0       # Krok prodloužení po zaznamenaném pohybu >= 20 cm
MAX_WAIT_SEC = 60.0              # Maximální celková doba čekání (1 minuta)
MOTION_SHIFT_THRESHOLD_M = 0.20  # Práh pohybu překážky (20 cm) pro vyhodnocení jako dynamická
EARLY_CLEAR_HOLD_SEC = 1.0       # Souvislé uvolnění trasy (1.0 s) pro okamžitý rozjezd
NO_PATH_CLEAR_HOLD_SEC = 1.0     # Hystereze uvolnění neprůjezdné cesty (1.0 s)

# Parametry zametené oblasti (swept footprint) a poloměrů překážek
LIVE_OBS_HARD_RADIUS_M = 0.48    # Tvrdý poloměr kolem 3D bodu pro CoR (37.5 cm šasi + ~10.5 cm rezerva)
LIVE_OBS_REPULSE_RADIUS_M = 1.15 # Měkký odtlačovací polštář kolem živé 3D překážky
SWEPT_GRID_RES_M = 0.20          # Rozlišení ENU mřížky zametené oblasti (20 cm)
SWEPT_INFLATE_RADIUS_M = 0.62    # Poloměr zápisu do zametené oblasti (šasi robota + poloměr pohybující se překážky)
SWEPT_CELL_COST = 92.0           # Vysoká penalizace (< 100) historické zametené oblasti
SWEPT_DECAY_SEC = 20.0           # Doba vyprchání zametené stopy po skončení čekání

# Parametry objíždění a předjíždění chodce ve stejném směru
AVOID_MAX_SPEED_CMS = 100.0      # Max rychlost při objíždění statické/usazené překážky (1.0 m/s)
STATIC_PASS_CLEAR_M = 1.60       # Náskok za statickou překážkou před návratem doprava
FOLLOW_MIN_V_PARALLEL_MS = 0.30  # Min dopředná rychlost chodce ve stejném směru (0.3 m/s)
FOLLOW_MAX_LAT_SPAN_M = 0.45     # Max příčné kličkování chodce pro klidné sledování/předjetí
FOLLOW_TARGET_GAP_M = 2.00       # Udržovaný odstup při sledování chodce (2.0 m)
FOLLOW_OBSERVE_SEC = 2.50        # Doba sledování chodce před zahájením předjíždění (2.5 s)
OVERTAKE_MIN_SPEED_DIFF_MS = 0.60  # Min rozdíl rychlostí (v_max - v_ped >= 0.6 m/s) pro předjetí
OVERTAKE_MIN_LAT_SEP_M = 0.95    # Min boční odstup středů při předjíždění chodce
OVERTAKE_MERGE_AHEAD_M = 2.20    # Bezpečný náskok před predikovanou polohou chodce při návratu doprava
OVERTAKE_MAX_DURATION_SEC = 22.0 # Časová pojistka předjížděcího manévru

# Práh obsazeného koncového cíle
GOAL_OCCUPIED_RADIUS_M = 2.00


@dataclass
class CenterlineSample:
    s: float
    e: float
    n: float
    te: float      # Jednotková tečna (East)
    tn: float      # Jednotková tečna (North)
    nr_e: float    # Jednotková pravostranná normála (East) = tn
    nr_n: float    # Jednotková pravostranná normála (North) = -te
    seg_idx: int


@dataclass
class ObstacleCluster:
    centroid_e: float
    centroid_n: float
    s_center: float
    s_min: float
    s_max: float
    d_lat_center: float
    d_lat_min: float
    d_lat_max: float
    min_dist_m: float
    points_enu: List[Tuple[float, float]] = field(default_factory=list)


@dataclass
class AvoidanceDecision:
    nav_state: str               # CRUISING, OBSTACLE, FOLLOWING, WAITING_CLEARANCE, AVOIDING, OVERTAKING, NO_PATH
    nav_info: str                # Lidsky čitelný popis stavu pro terminál a logy
    target_heading_deg: float    # Požadovaný azimut z plánovače koridoru (0..360 CW)
    speed_cap_cms: float         # Omezení rychlosti z logiky překážek [cm/s]
    lateral_offset_m: float      # Cílové boční vybočení v koridoru (-vlevo, +vpravo) [m]
    path_found: bool             # Zda v koridoru existuje průjezdná cesta (cost < 100)
    goal_reached_near_obstacle: bool = False  # True pokud statická překážka stojí přímo na cíli (<= 2 m)


class RouteCorridor:
    """
    Spojitá reprezentace referenční trasy v lokální ENU soustavě se zaoblením rohů křižovatek.
    Zajišťuje, že koridor [-2 m vlevo, +1 m vpravo] plynule zatáčí už před samotnou křižovatkou.
    """

    def __init__(self, waypoints: list, sample_step_m: float = 0.20):
        if len(waypoints) < 2:
            raise ValueError("RouteCorridor vyžaduje alespoň 2 waypointy")

        self.lat0 = float(waypoints[0].lat)
        self.lon0 = float(waypoints[0].lon)
        lat_rad = deg2rad(self.lat0)
        self._m_per_deg_lat = 111132.92 - 559.82 * math.cos(2.0 * lat_rad) + 1.175 * math.cos(4.0 * lat_rad)
        self._m_per_deg_lon = 111412.84 * math.cos(lat_rad) - 93.5 * math.cos(3.0 * lat_rad)

        self.wp_enu: List[Tuple[float, float]] = [
            self.wgs84_to_enu(wp.lat, wp.lon) for wp in waypoints
        ]
        self.sample_step_m = sample_step_m
        self.samples: List[CenterlineSample] = []
        self._seg_bounds: Dict[int, Tuple[int, int]] = {}
        self._sample_s: List[float] = []
        self._build_filleted_centerline()
        self.total_length_m = self.samples[-1].s if self.samples else 0.0

    def wgs84_to_enu(self, lat: float, lon: float) -> Tuple[float, float]:
        e = (float(lon) - self.lon0) * self._m_per_deg_lon
        n = (float(lat) - self.lat0) * self._m_per_deg_lat
        return e, n

    def enu_to_wgs84(self, e: float, n: float) -> Tuple[float, float]:
        lat = self.lat0 + (n / self._m_per_deg_lat)
        lon = self.lon0 + (e / self._m_per_deg_lon)
        return lat, lon

    def body_to_enu(self, e_rrp: float, n_rrp: float, heading_deg: float, x_fwd_m: float, y_cw_m: float) -> Tuple[float, float]:
        psi = deg2rad(heading_deg)
        sin_p = math.sin(psi)
        cos_p = math.cos(psi)
        e = e_rrp + x_fwd_m * sin_p + y_cw_m * cos_p
        n = n_rrp + x_fwd_m * cos_p - y_cw_m * sin_p
        return e, n

    def enu_to_body(self, e_rrp: float, n_rrp: float, heading_deg: float, e_pt: float, n_pt: float) -> Tuple[float, float]:
        psi = deg2rad(heading_deg)
        sin_p = math.sin(psi)
        cos_p = math.cos(psi)
        de = e_pt - e_rrp
        dn = n_pt - n_rrp
        x_fwd = de * sin_p + dn * cos_p
        y_cw = de * cos_p - dn * sin_p
        return x_fwd, y_cw

    def _build_filleted_centerline(self):
        """Sestaví hladkou lomenou čáru s kvadratickým Bézierovým zaoblením vnitřních rohů."""
        m = len(self.wp_enu)
        # Spočítáme délky a směry úseků
        seg_vecs: List[Tuple[float, float, float]] = []
        for i in range(m - 1):
            dx = self.wp_enu[i + 1][0] - self.wp_enu[i][0]
            dy = self.wp_enu[i + 1][1] - self.wp_enu[i][1]
            length = math.hypot(dx, dy)
            if length > 1e-6:
                seg_vecs.append((dx / length, dy / length, length))
            else:
                seg_vecs.append((0.0, 1.0, 0.0))

        # Ořezové vzdálenosti v rozích pro hladký přechod normál koridoru
        trim_in = [0.0] * m
        trim_out = [0.0] * m
        for i in range(1, m - 1):
            u1x, u1y, len1 = seg_vecs[i - 1]
            u2x, u2y, len2 = seg_vecs[i]
            if len1 < 0.3 or len2 < 0.3:
                continue
            dot = max(-1.0, min(1.0, u1x * u2x + u1y * u2y))
            turn_rad = math.acos(dot)
            if turn_rad > math.radians(3.0):
                d_trim = min(1.4, 0.35 * len1, 0.35 * len2)
                trim_in[i] = d_trim
                trim_out[i] = d_trim

        # Sestavíme hustou posloupnost uzlů (e, n, seg_idx)
        raw_pts: List[Tuple[float, float, int]] = []
        for i in range(m - 1):
            ux, uy, seg_len = seg_vecs[i]
            if seg_len < 1e-6:
                continue
            s_start = trim_out[i]
            s_end = max(s_start, seg_len - trim_in[i + 1])

            p_start = (
                self.wp_enu[i][0] + ux * s_start,
                self.wp_enu[i][1] + uy * s_start,
            )
            p_end = (
                self.wp_enu[i][0] + ux * s_end,
                self.wp_enu[i][1] + uy * s_end,
            )

            straight_len = max(0.0, s_end - s_start)
            n_steps = max(1, int(math.ceil(straight_len / self.sample_step_m)))
            start_k = 0 if not raw_pts else 1
            for k in range(start_k, n_steps + 1):
                alpha = k / n_steps
                px = p_start[0] + alpha * (p_end[0] - p_start[0])
                py = p_start[1] + alpha * (p_end[1] - p_start[1])
                raw_pts.append((px, py, i))

            # Přidáme zaoblený oblouk kolem vnitřního vrcholu i+1
            if i + 1 < m - 1 and trim_in[i + 1] > 0.05 and trim_out[i + 1] > 0.05:
                u2x, u2y, _ = seg_vecs[i + 1]
                vx, vy = self.wp_enu[i + 1]
                b0 = p_end
                b1 = (vx, vy)
                b2 = (vx + u2x * trim_out[i + 1], vy + u2y * trim_out[i + 1])
                arc_approx = math.hypot(b1[0] - b0[0], b1[1] - b0[1]) + math.hypot(b2[0] - b1[0], b2[1] - b1[1])
                arc_steps = max(2, int(math.ceil(arc_approx / (0.5 * self.sample_step_m))))
                for k in range(1, arc_steps + 1):
                    t = k / arc_steps
                    omt = 1.0 - t
                    bx = omt * omt * b0[0] + 2.0 * omt * t * b1[0] + t * t * b2[0]
                    by = omt * omt * b0[1] + 2.0 * omt * t * b1[1] + t * t * b2[1]
                    seg_tag = i if t < 0.5 else (i + 1)
                    raw_pts.append((bx, by, seg_tag))

        if not raw_pts:
            e0, n0 = self.wp_enu[0]
            self.samples = [CenterlineSample(0.0, e0, n0, 0.0, 1.0, 1.0, 0.0, 0)]
            self._seg_bounds = {0: (0, 0)}
            self._sample_s = [0.0]
            return

        # Spočítáme kumulativní délku oblouku a vyhlazené tečny/normály
        cum_s = [0.0]
        for idx in range(1, len(raw_pts)):
            ds = math.hypot(raw_pts[idx][0] - raw_pts[idx - 1][0], raw_pts[idx][1] - raw_pts[idx - 1][1])
            cum_s.append(cum_s[-1] + ds)

        n_raw = len(raw_pts)
        for idx in range(n_raw):
            prev_i = max(0, idx - 1)
            next_i = min(n_raw - 1, idx + 1)
            tx = raw_pts[next_i][0] - raw_pts[prev_i][0]
            ty = raw_pts[next_i][1] - raw_pts[prev_i][1]
            tlen = math.hypot(tx, ty)
            if tlen < 1e-9:
                te, tn = 0.0, 1.0
            else:
                te, tn = tx / tlen, ty / tlen
            seg_i = raw_pts[idx][2]
            # Pravostranná normála v ENU: otočení (te, tn) o 90° CW -> (tn, -te)
            self.samples.append(
                CenterlineSample(
                    s=cum_s[idx],
                    e=raw_pts[idx][0],
                    n=raw_pts[idx][1],
                    te=te,
                    tn=tn,
                    nr_e=tn,
                    nr_n=-te,
                    seg_idx=seg_i,
                )
            )
            if seg_i not in self._seg_bounds:
                self._seg_bounds[seg_i] = (idx, idx)
            else:
                self._seg_bounds[seg_i] = (self._seg_bounds[seg_i][0], idx)

        self._sample_s = cum_s

    def project_enu(self, e_pt: float, n_pt: float, hint_seg_idx: Optional[int] = None) -> Tuple[float, float, int]:
        """
        Promítne ENU bod na referenční křivku trasy.
        Vrací: (s_m, d_lat_m, sample_idx), kde d_lat_m > 0 je vpravo a d_lat_m < 0 vlevo.
        """
        best_dist2 = float("inf")
        best_idx = 0

        if hint_seg_idx is not None and self._seg_bounds:
            idx_lo: Optional[int] = None
            idx_hi: Optional[int] = None
            for seg_k in range(hint_seg_idx - 2, hint_seg_idx + 3):
                bounds = self._seg_bounds.get(seg_k)
                if bounds is not None:
                    idx_lo = bounds[0] if idx_lo is None else min(idx_lo, bounds[0])
                    idx_hi = bounds[1] if idx_hi is None else max(idx_hi, bounds[1])
            if idx_lo is not None and idx_hi is not None:
                for idx in range(idx_lo, idx_hi + 1):
                    smp = self.samples[idx]
                    de = e_pt - smp.e
                    dn = n_pt - smp.n
                    d2 = de * de + dn * dn
                    if d2 < best_dist2:
                        best_dist2 = d2
                        best_idx = idx

        # Fallback přes celou trasu, pokud okno kolem hint_seg_idx nebylo zadáno nebo nic nenašlo
        if best_dist2 == float("inf"):
            for idx, smp in enumerate(self.samples):
                de = e_pt - smp.e
                dn = n_pt - smp.n
                d2 = de * de + dn * dn
                if d2 < best_dist2:
                    best_dist2 = d2
                    best_idx = idx

        smp = self.samples[best_idx]
        de = e_pt - smp.e
        dn = n_pt - smp.n
        s_proj = smp.s + (de * smp.te + dn * smp.tn)
        s_proj = max(0.0, min(self.total_length_m, s_proj))
        d_lat = de * smp.nr_e + dn * smp.nr_n
        return s_proj, d_lat, best_idx

    def sample_at_s(self, s_query: float, start_idx: int = 0) -> CenterlineSample:
        """Lineárně interpoluje vzorek křivky v obloukové vzdálenosti s_query."""
        if not self.samples:
            return CenterlineSample(0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 0.0, 0)
        if s_query <= 0.0:
            return self.samples[0]
        if s_query >= self.total_length_m:
            return self.samples[-1]

        idx = bisect.bisect_right(self._sample_s, s_query) - 1
        idx = max(0, min(len(self.samples) - 2, idx))

        s0 = self.samples[idx]
        s1 = self.samples[idx + 1]
        span = s1.s - s0.s
        if span < 1e-6:
            return s0
        alpha = max(0.0, min(1.0, (s_query - s0.s) / span))
        te = s0.te + alpha * (s1.te - s0.te)
        tn = s0.tn + alpha * (s1.tn - s0.tn)
        tlen = math.hypot(te, tn)
        if tlen > 1e-9:
            te /= tlen
            tn /= tlen
        else:
            te, tn = s0.te, s0.tn
        return CenterlineSample(
            s=s_query,
            e=s0.e + alpha * (s1.e - s0.e),
            n=s0.n + alpha * (s1.n - s0.n),
            te=te,
            tn=tn,
            nr_e=tn,
            nr_n=-te,
            seg_idx=s0.seg_idx,
        )


class ObstacleAvoidanceController:
    """
    Řídí chování pilota v koridoru [-2 m vlevo, +1 m vpravo]:
    - Plynulé vyhýbání hrubému terénu / nízkým terénním vadám (LIDAR-TRAVERSABILITY) za jízdy.
    - Zpomalení a zastavení (~1.45 m) u 3D překážek (LIDAR-COLLISIONS), čekání 5 s.
    - Detekce pohybu překážky (>= 20 cm) -> prodlužování čekání po +5 s až do 60 s + sběr zametené oblasti.
    - Předjíždění chodce jdoucího stejným směrem (srovnání rychlosti 2.5 s -> předjetí při rozdílu >= 0.6 m/s
      s predikcí chodce ve slepém úhlu >= 2.2 m před návratem do pravého pruhu).
    - Stav NO_PATH při zablokovaném koridoru a automatické pokračování po uvolnění.
    """

    def __init__(self, corridor: RouteCorridor):
        self.corridor = corridor

        # Diskrétní boční posuny v koridoru [-2.0 m .. +1.0 m] po 10 cm (31 hodnot)
        n_lat = int(round((CORRIDOR_LEFT_M + CORRIDOR_RIGHT_M) / LATERAL_STEP_M)) + 1
        self.lat_offsets: List[float] = [
            round(-CORRIDOR_LEFT_M + j * LATERAL_STEP_M, 3) for j in range(n_lat)
        ]

        self.nav_state: str = "CRUISING"
        self.nav_info: str = "Jízda podle trasy"
        self.prev_plan_offsets: List[float] = []

        # Sledování primární 3D překážky v ENU [(ts, e, n, s, d_lat), ...]
        self._obs_history: List[Tuple[float, float, float, float, float]] = []

        # Časovače čekání u překážky
        self._wait_start_ts: float = 0.0
        self._wait_deadline_ts: float = 0.0
        self._is_dynamic_obs: bool = False
        self._last_motion_ref_enu: Optional[Tuple[float, float]] = None
        self._clear_since_ts: Optional[float] = None

        # Zametená oblast (swept footprint) v 20cm ENU buňkách: (gx, gy) -> expire_or_seen_ts
        self.swept_cells: Dict[Tuple[int, int], float] = {}
        self._swept_frozen_at_ts: Optional[float] = None

        # Paměť statické překážky při objíždění (ochrana zadních kol ve slepém úhlu)
        self._avoid_target_s: float = 0.0
        self._avoid_target_d_lat: float = 0.0
        self._avoid_active_obs_enu: Optional[Tuple[float, float]] = None

        # Sledování a předjíždění chodce ve stejném směru
        self._follow_start_ts: float = 0.0
        self._ped_last_s: float = 0.0
        self._ped_last_d_lat: float = 0.0
        self._ped_v_parallel: float = 0.0
        self._ped_last_seen_ts: float = 0.0
        self._overtake_start_ts: float = 0.0

        # Hystereze stavu NO_PATH
        self._no_path_clear_since_ts: Optional[float] = None

    def extract_3d_obstacles(
        self,
        e_rrp: float,
        n_rrp: float,
        heading_deg: float,
        s_robot: float,
        hint_seg_idx: int,
        collision_data: Optional[dict],
    ) -> Tuple[List[Tuple[float, float, float, float, float]], List[ObstacleCluster]]:
        """
        Převede 3D sektory z LIDAR-COLLISIONS (přední polokoule 0..17 a 54..71) do globálního ENU
        a seskupí je do shluků v okolí koridoru.
        Vrací: (raw_points[(e, n, s, d_lat, dist_m)], clusters)
        """
        if not collision_data:
            return [], []
        distances = collision_data.get("distances_cm")
        if not distances or len(distances) != 72:
            return [], []

        pts: List[Tuple[float, float, float, float, float]] = []
        sector_indices = list(range(0, 18)) + list(range(54, 72))
        for b in sector_indices:
            d_cm = float(distances[b])
            if d_cm <= 0.0 or d_cm >= 650.0:
                continue
            d_m = d_cm * 0.01
            ang_rad = math.radians((b + 0.5) * 5.0)
            x_fwd = d_m * math.cos(ang_rad)
            y_cw = d_m * math.sin(ang_rad)
            if x_fwd < -0.20:
                continue
            oe, on = self.corridor.body_to_enu(e_rrp, n_rrp, heading_deg, x_fwd, y_cw)
            s_obs, d_lat_obs, _ = self.corridor.project_enu(oe, on, hint_seg_idx=hint_seg_idx)
            if (s_robot - 0.30) <= s_obs <= (s_robot + 6.20):
                if (-CORRIDOR_LEFT_M - 0.80) <= d_lat_obs <= (CORRIDOR_RIGHT_M + 0.80):
                    pts.append((oe, on, s_obs, d_lat_obs, d_m))

        if not pts:
            return [], []

        # Shlukování blízkých 3D bodů (vzdálenost <= 0.65 m)
        used = [False] * len(pts)
        clusters: List[ObstacleCluster] = []
        for i in range(len(pts)):
            if used[i]:
                continue
            queue = [i]
            used[i] = True
            comp = []
            while queue:
                cur = queue.pop()
                comp.append(pts[cur])
                ce, cn = pts[cur][0], pts[cur][1]
                for j in range(len(pts)):
                    if not used[j]:
                        if math.hypot(pts[j][0] - ce, pts[j][1] - cn) <= 0.65:
                            used[j] = True
                            queue.append(j)

            n_c = len(comp)
            ce = sum(p[0] for p in comp) / n_c
            cn = sum(p[1] for p in comp) / n_c
            s_vals = [p[2] for p in comp]
            d_vals = [p[3] for p in comp]
            min_d = min(p[4] for p in comp)
            clusters.append(
                ObstacleCluster(
                    centroid_e=ce,
                    centroid_n=cn,
                    s_center=sum(s_vals) / n_c,
                    s_min=min(s_vals),
                    s_max=max(s_vals),
                    d_lat_center=sum(d_vals) / n_c,
                    d_lat_min=min(d_vals),
                    d_lat_max=max(d_vals),
                    min_dist_m=min_d,
                    points_enu=[(p[0], p[1]) for p in comp],
                )
            )

        clusters.sort(key=lambda c: c.s_min)
        return pts, clusters

    def _find_blocking_cluster_on_ideal_route(
        self, s_robot: float, clusters: List[ObstacleCluster], max_ahead_m: float = APPROACH_SLOW_DIST_M
    ) -> Optional[ObstacleCluster]:
        """Najde nejbližší 3D shluk zasahující do ideální stopy (d_lat = 0 ± LIVE_OBS_HARD_RADIUS_M)."""
        half_band = LIVE_OBS_HARD_RADIUS_M + 0.05
        for cl in clusters:
            ahead = cl.s_min - s_robot
            if -0.10 <= ahead <= max_ahead_m:
                if cl.d_lat_min <= half_band and cl.d_lat_max >= -half_band:
                    return cl
        return None

    def _update_obstacle_kinematics(self, now: float, cluster: Optional[ObstacleCluster]) -> Tuple[float, float, float]:
        """
        Aktualizuje historii polohy sledované překážky a spočítá:
        (v_parallel_ms, v_perp_ms, lat_span_m) v souřadnicích trasy.
        """
        if cluster is None:
            self._obs_history = [h for h in self._obs_history if (now - h[0]) <= 1.2]
            return 0.0, 0.0, 0.0

        if self._obs_history:
            last_t, last_e, last_n, _, _ = self._obs_history[-1]
            dt_last = max(0.05, now - last_t)
            # Pokud jde o úplně jiný vzdálený objekt, resetujeme historii rychlosti
            if math.hypot(cluster.centroid_e - last_e, cluster.centroid_n - last_n) > max(1.2, 3.5 * dt_last):
                self._obs_history.clear()

        self._obs_history.append((now, cluster.centroid_e, cluster.centroid_n, cluster.s_center, cluster.d_lat_center))
        self._obs_history = [h for h in self._obs_history if (now - h[0]) <= 2.5]

        if len(self._obs_history) < 2:
            return 0.0, 0.0, 0.0

        t0, _, _, s0, d0 = self._obs_history[0]
        t1, _, _, s1, d1 = self._obs_history[-1]
        dt = t1 - t0
        if dt < 0.25:
            return 0.0, 0.0, 0.0

        v_par = (s1 - s0) / dt
        v_perp = (d1 - d0) / dt
        d_all = [h[4] for h in self._obs_history]
        lat_span = max(d_all) - min(d_all)
        return v_par, v_perp, lat_span

    def _record_swept_footprint(self, points_enu: List[Tuple[float, float]], now: float):
        """Zapíše 3D body překážky (nafouknuté o poloměr robota) do paměti zametené oblasti."""
        rad_cells = int(math.ceil(SWEPT_INFLATE_RADIUS_M / SWEPT_GRID_RES_M))
        r2_max = SWEPT_INFLATE_RADIUS_M * SWEPT_INFLATE_RADIUS_M
        for pe, pn in points_enu:
            gx0 = int(math.floor(pe / SWEPT_GRID_RES_M))
            gy0 = int(math.floor(pn / SWEPT_GRID_RES_M))
            for dx in range(-rad_cells, rad_cells + 1):
                for dy in range(-rad_cells, rad_cells + 1):
                    ce = (gx0 + dx + 0.5) * SWEPT_GRID_RES_M
                    cn = (gy0 + dy + 0.5) * SWEPT_GRID_RES_M
                    if (ce - pe) ** 2 + (cn - pn) ** 2 <= r2_max:
                        self.swept_cells[(gx0 + dx, gy0 + dy)] = now

    def _prune_swept_cells(self, now: float):
        """Postupně vymaže starou zametenou oblast po uplynutí SWEPT_DECAY_SEC od konce čekání."""
        if not self.swept_cells:
            return
        if self._swept_frozen_at_ts is not None and (now - self._swept_frozen_at_ts) > SWEPT_DECAY_SEC:
            self.swept_cells.clear()
            self._swept_frozen_at_ts = None

    def plan_corridor_path(
        self,
        e_rrp: float,
        n_rrp: float,
        heading_deg: float,
        s_robot: float,
        d_lat_robot: float,
        start_sample_idx: int,
        traversability_active: bool,
        get_traversability_cost_fn: Callable[[float, float], int],
        live_obs_points_enu: List[Tuple[float, float]],
        virtual_hard_zones_enu: List[Tuple[float, float, float]],
        allow_3d_detour: bool,
        preferred_min_sep_from_ped: Optional[Tuple[float, float]] = None,
    ) -> Tuple[bool, List[Tuple[float, float, float]], float]:
        """
        Vyhledá optimální plynulou trajektorii (pružnou gumu / DP) v koridoru [-2.0 m, +1.0 m].
        Vrací: (path_found, [(s_k, e_k, n_k), ...], target_d_lat_m)
        """
        rem_len = max(0.0, self.corridor.total_length_m - s_robot)
        horizon = min(LOOKAHEAD_HORIZON_M, max(0.50, rem_len))
        n_slices = max(2, int(math.floor(horizon / SLICE_STEP_M)) + 1)
        n_lat = len(self.lat_offsets)

        INF = 1e12
        dp = [[INF] * n_lat for _ in range(n_slices)]
        parent = [[-1] * n_lat for _ in range(n_slices)]
        slice_samples: List[CenterlineSample] = [
            self.corridor.sample_at_s(min(self.corridor.total_length_m, s_robot + k * SLICE_STEP_M), start_sample_idx)
            for k in range(n_slices)
        ]

        # Krok k = 0: ukotvení u aktuálního bočního posunu robota d_lat_robot
        clamped_robot_d = max(-CORRIDOR_LEFT_M, min(CORRIDOR_RIGHT_M, d_lat_robot))
        for j, dj in enumerate(self.lat_offsets):
            if abs(dj - clamped_robot_d) <= 0.25:
                dp[0][j] = 20.0 * abs(dj - clamped_robot_d)

        max_step_cells = 3  # max 0.30 m příčně na 0.25 m podélného kroku (~50°)

        for k in range(1, n_slices):
            smp = slice_samples[k]
            prev_pref = self.prev_plan_offsets[k] if k < len(self.prev_plan_offsets) else 0.0

            for j, dj in enumerate(self.lat_offsets):
                # Pokud neobjíždíme 3D překážku (např. jen brzdíme k překážce před vyčkáním),
                # držíme se blízko ideální stopy (pokud nejde o korekci terénu bez 3D překážky)
                if not allow_3d_detour and abs(dj - clamped_robot_d) > 0.35 and abs(dj) > 0.35:
                    continue

                pe = smp.e + dj * smp.nr_e
                pn = smp.n + dj * smp.nr_n

                # 1. Kontrola živých 3D překážek v ENU
                c_obs = 0.0
                blocked = False
                for oe, on in live_obs_points_enu:
                    dist_o = math.hypot(pe - oe, pn - on)
                    if dist_o <= LIVE_OBS_HARD_RADIUS_M:
                        blocked = True
                        break
                    elif dist_o < LIVE_OBS_REPULSE_RADIUS_M:
                        ratio = (LIVE_OBS_REPULSE_RADIUS_M - dist_o) / (
                            LIVE_OBS_REPULSE_RADIUS_M - LIVE_OBS_HARD_RADIUS_M
                        )
                        rep = 65.0 * ratio * ratio
                        if rep > c_obs:
                            c_obs = rep

                if blocked:
                    continue

                # 2. Virtuální zakázané zóny (např. chodec ve slepém úhlu při předjíždění / bok statické překážky)
                for ve, vn, vrad in virtual_hard_zones_enu:
                    if math.hypot(pe - ve, pn - vn) <= vrad:
                        blocked = True
                        break
                if blocked:
                    continue

                # Při předjíždění chodce vyžadujeme bezpečný boční odstup v oblasti míjení a slepého úhlu
                if preferred_min_sep_from_ped is not None:
                    s_ped_now, d_ped_now = preferred_min_sep_from_ped
                    needed_shift_m = max(0.0, OVERTAKE_MIN_LAT_SEP_M - abs(clamped_robot_d - d_ped_now))
                    s_reachable = s_robot + (needed_shift_m / 0.80) + 0.25
                    s_enforce_min = max(s_ped_now - 0.60, s_reachable)
                    if s_robot < (s_ped_now + OVERTAKE_MERGE_AHEAD_M):
                        s_enforce_max = max(s_ped_now + OVERTAKE_MERGE_AHEAD_M, s_robot + 2.50)
                    else:
                        s_enforce_max = s_ped_now + OVERTAKE_MERGE_AHEAD_M
                    if s_enforce_min <= smp.s <= s_enforce_max:
                        if abs(dj - d_ped_now) < OVERTAKE_MIN_LAT_SEP_M:
                            continue

                # 3. Kontrola terénu z LIDAR-TRAVERSABILITY
                c_trav = 0.0
                if traversability_active:
                    x_fwd, y_cw = self.corridor.enu_to_body(e_rrp, n_rrp, heading_deg, pe, pn)
                    if -1.8 <= x_fwd < 5.8 and -3.8 <= y_cw < 3.8:
                        raw_tc = get_traversability_cost_fn(x_fwd, y_cw)
                        # Spolehlivé okno pro tvrdou stopku povrchu (mimo ignoreBox pod robotem a vzdálený řídký horizont)
                        if raw_tc >= 100:
                            if 0.65 <= x_fwd <= 4.20 and abs(y_cw) <= 3.50:
                                continue
                        else:
                            c_trav = float(raw_tc)

                # 4. Historická zametená oblast (swept zone) - vysoká penalizace 92, ale průjezdná v nouzi
                c_swept = 0.0
                if self.swept_cells:
                    gx = int(math.floor(pe / SWEPT_GRID_RES_M))
                    gy = int(math.floor(pn / SWEPT_GRID_RES_M))
                    if (gx, gy) in self.swept_cells:
                        c_swept = SWEPT_CELL_COST

                # Živé odtlačení c_obs přičítáme nad rámec povrchu/zametené stopy, aby ani v zametené
                # oblasti robot netěsnil živou osobu na hranici tvrdého poloměru
                c_env = max(c_trav, c_swept) + c_obs

                # Přitažlivost k ideální stopě (d=0) + mírná preference levé strany (dj < 0) pro předjíždění
                right_bias = 3.0 * dj if dj > 0.0 else 0.0
                c_lat = 3.5 * abs(dj) + 2.0 * (dj * dj) + right_bias
                c_hyst = 1.2 * abs(dj - prev_pref)

                node_cost = c_env + c_lat + c_hyst

                j_min = max(0, j - max_step_cells)
                j_max = min(n_lat - 1, j + max_step_cells)
                best_prev_cost = INF
                best_prev_j = -1
                for jp in range(j_min, j_max + 1):
                    if dp[k - 1][jp] >= INF:
                        continue
                    slope = (dj - self.lat_offsets[jp]) / SLICE_STEP_M
                    trans_cost = 8.0 * (slope * slope)
                    cand = dp[k - 1][jp] + node_cost + trans_cost
                    if cand < best_prev_cost:
                        best_prev_cost = cand
                        best_prev_j = jp

                dp[k][j] = best_prev_cost
                parent[k][j] = best_prev_j

        # Nalezení nejlepší koncové buňky v řezu n_slices - 1
        last_k = n_slices - 1
        best_j = -1
        best_total = INF
        for j in range(n_lat):
            if dp[last_k][j] < best_total:
                best_total = dp[last_k][j]
                best_j = j

        if best_j < 0 or best_total >= INF:
            return False, [], 0.0

        # Rekonstrukce optimální pružné gumy
        idx_path = [0] * n_slices
        cur_j = best_j
        for k in range(last_k, -1, -1):
            idx_path[k] = cur_j
            cur_j = parent[k][cur_j] if k > 0 else cur_j

        self.prev_plan_offsets = [self.lat_offsets[j] for j in idx_path]
        pts_out: List[Tuple[float, float, float]] = []
        for k in range(n_slices):
            smp = slice_samples[k]
            dj = self.lat_offsets[idx_path[k]]
            pts_out.append((smp.s, smp.e + dj * smp.nr_e, smp.n + dj * smp.nr_n))

        # Reprezentativní cílové boční vybočení v aktivním okně manévru (~0.25..2.50 m před robotem)
        window_end_k = min(last_k, 10)
        peak_k = max(range(1, window_end_k + 1), key=lambda k: abs(self.lat_offsets[idx_path[k]]))
        target_d_lat = self.lat_offsets[idx_path[peak_k]]
        return True, pts_out, target_d_lat

    @staticmethod
    def _compute_lookahead_heading(
        e_ref: float,
        n_ref: float,
        s_robot: float,
        path_pts: List[Tuple[float, float, float]],
        lookahead_m: float,
        fallback_heading_deg: float,
    ) -> float:
        """Spočítá azimut (GNSS CW stupně) z referenčního bodu robota na bod pružné gumy ve vzdálenosti lookahead_m."""
        if not path_pts:
            return fallback_heading_deg

        target_s = s_robot + lookahead_m
        chosen_e, chosen_n = path_pts[-1][1], path_pts[-1][2]
        for idx in range(len(path_pts) - 1):
            s0, e0, n0 = path_pts[idx]
            s1, e1, n1 = path_pts[idx + 1]
            if s1 >= target_s:
                span = max(1e-6, s1 - s0)
                alpha = max(0.0, min(1.0, (target_s - s0) / span))
                chosen_e = e0 + alpha * (e1 - e0)
                chosen_n = n0 + alpha * (n1 - n0)
                break

        de = chosen_e - e_ref
        dn = chosen_n - n_ref
        if math.hypot(de, dn) < 0.05:
            return fallback_heading_deg
        psi_enu_deg = math.degrees(math.atan2(dn, de))
        return heading_enu_to_gnss(psi_enu_deg)

    @staticmethod
    def _format_side_offset(d_lat_m: float) -> str:
        if d_lat_m < -0.15:
            return f"vlevo ({abs(d_lat_m):.1f} m)"
        elif d_lat_m > 0.15:
            return f"vpravo ({abs(d_lat_m):.1f} m)"
        return "ve stopě"

    def step(
        self,
        lat_rrp: float,
        lon_rrp: float,
        heading_deg: float,
        current_speed_cms: float,
        max_speed_cms: float,
        current_wp_index: int,
        default_target_heading_deg: float,
        collision_data: Optional[dict],
        collisions_active: bool,
        traversability_active: bool,
        get_traversability_cost_fn: Callable[[float, float], int],
        now: Optional[float] = None,
    ) -> AvoidanceDecision:
        """Hlavní 10Hz krok vyhodnocení překážek, stavového automatu a lokálního plánování koridoru."""
        now_ts = time.time() if now is None else float(now)
        self._prune_swept_cells(now_ts)

        e_rrp, n_rrp = self.corridor.wgs84_to_enu(lat_rrp, lon_rrp)
        s_robot, d_lat_robot, smp_idx = self.corridor.project_enu(e_rrp, n_rrp, hint_seg_idx=current_wp_index)
        rem_goal_m = max(0.0, self.corridor.total_length_m - s_robot)
        max_speed_ms = max_speed_cms / 100.0

        raw_obs_pts, clusters = (
            self.extract_3d_obstacles(e_rrp, n_rrp, heading_deg, s_robot, current_wp_index, collision_data)
            if collisions_active
            else ([], [])
        )
        live_obs_enu = [(p[0], p[1]) for p in raw_obs_pts]

        blocking_cl = self._find_blocking_cluster_on_ideal_route(s_robot, clusters, max_ahead_m=APPROACH_SLOW_DIST_M)
        v_par, v_perp, lat_span = self._update_obstacle_kinematics(now_ts, blocking_cl)

        # ------------------------------------------------------------------
        # STAVOVÝ AUTOMAT PŘEKÁŽEK
        # ------------------------------------------------------------------

        # 1. CRUISING -> detekce 3D překážky na ideální trase
        if self.nav_state == "CRUISING":
            if blocking_cl is not None:
                if v_par >= FOLLOW_MIN_V_PARALLEL_MS and lat_span <= FOLLOW_MAX_LAT_SPAN_M:
                    self.nav_state = "FOLLOWING"
                    self._follow_start_ts = now_ts
                    self._ped_last_s = blocking_cl.s_center
                    self._ped_last_d_lat = blocking_cl.d_lat_center
                    self._ped_v_parallel = v_par
                    self._ped_last_seen_ts = now_ts
                else:
                    self.nav_state = "OBSTACLE"
                    self._clear_since_ts = None

        # 2. OBSTACLE (zpomalování a dojíždění k zastavení ~1.45 m před 3D překážkou)
        if self.nav_state == "OBSTACLE":
            if blocking_cl is None:
                if self._clear_since_ts is None:
                    self._clear_since_ts = now_ts
                elif (now_ts - self._clear_since_ts) >= EARLY_CLEAR_HOLD_SEC:
                    self.nav_state = "CRUISING"
                    self._clear_since_ts = None
            else:
                self._clear_since_ts = None
                self._record_swept_footprint(blocking_cl.points_enu, now_ts)
                dist_ahead = max(0.0, blocking_cl.min_dist_m)

                # Pokud se během dojíždění ukázalo, že jde o chodce jdoucího stejným směrem -> plynulé sledování
                if (
                    len(self._obs_history) >= 4
                    and v_par >= FOLLOW_MIN_V_PARALLEL_MS
                    and lat_span <= FOLLOW_MAX_LAT_SPAN_M
                    and dist_ahead > WAIT_STOP_DIST_M
                ):
                    self.nav_state = "FOLLOWING"
                    self._follow_start_ts = now_ts
                    self._ped_last_s = blocking_cl.s_center
                    self._ped_last_d_lat = blocking_cl.d_lat_center
                    self._ped_v_parallel = v_par
                    self._ped_last_seen_ts = now_ts
                    self.swept_cells.clear()
                    self._swept_frozen_at_ts = None
                elif dist_ahead <= (WAIT_STOP_DIST_M + 0.10) or (
                    dist_ahead <= 2.10 and current_speed_cms <= 8.0
                ):
                    # Zastaveno u překážky -> zahajujeme 5s vyčkávání a sledování pohybu
                    self.nav_state = "WAITING_CLEARANCE"
                    self._wait_start_ts = now_ts
                    self._wait_deadline_ts = now_ts + BASE_WAIT_SEC
                    self._is_dynamic_obs = False
                    self._last_motion_ref_enu = (blocking_cl.centroid_e, blocking_cl.centroid_n)
                    self._clear_since_ts = None
                    self.swept_cells.clear()
                    self._swept_frozen_at_ts = None
                    self._record_swept_footprint(blocking_cl.points_enu, now_ts)

        # 3. WAITING_CLEARANCE (čekání 5 s, při pohybu >= 20 cm prodlužování až do 60 s)
        if self.nav_state == "WAITING_CLEARANCE":
            near_blocking = self._find_blocking_cluster_on_ideal_route(s_robot, clusters, max_ahead_m=3.60)
            if near_blocking is None:
                if self._clear_since_ts is None:
                    self._clear_since_ts = now_ts
                elif (now_ts - self._clear_since_ts) >= EARLY_CLEAR_HOLD_SEC:
                    # Překážka sama odešla z trasy -> ihned pokračujeme po ideální trase!
                    self.swept_cells.clear()
                    self._swept_frozen_at_ts = None
                    self.nav_state = "CRUISING"
                    self._clear_since_ts = None
            else:
                self._clear_since_ts = None
                self._record_swept_footprint(near_blocking.points_enu, now_ts)
                self._avoid_target_s = near_blocking.s_max
                self._avoid_target_d_lat = near_blocking.d_lat_center
                self._avoid_active_obs_enu = (near_blocking.centroid_e, near_blocking.centroid_n)

                if self._last_motion_ref_enu is None:
                    self._last_motion_ref_enu = (near_blocking.centroid_e, near_blocking.centroid_n)
                else:
                    shift_m = math.hypot(
                        near_blocking.centroid_e - self._last_motion_ref_enu[0],
                        near_blocking.centroid_n - self._last_motion_ref_enu[1],
                    )
                    if shift_m >= MOTION_SHIFT_THRESHOLD_M:
                        self._is_dynamic_obs = True
                        self._last_motion_ref_enu = (near_blocking.centroid_e, near_blocking.centroid_n)
                        hard_cap_ts = self._wait_start_ts + MAX_WAIT_SEC
                        self._wait_deadline_ts = min(
                            hard_cap_ts,
                            max(self._wait_deadline_ts, now_ts + EXTEND_WAIT_STEP_SEC),
                        )

                # Vypršel čas čekání (5 s klidu nebo 60 s limit) -> vyhodnotíme cíl nebo objížďku
                if now_ts >= self._wait_deadline_ts:
                    obs_to_goal_m = max(0.0, self.corridor.total_length_m - near_blocking.s_center)
                    if rem_goal_m <= GOAL_OCCUPIED_RADIUS_M or (
                        obs_to_goal_m <= GOAL_OCCUPIED_RADIUS_M and rem_goal_m <= 2.80
                    ):
                        return AvoidanceDecision(
                            nav_state="CRUISING",
                            nav_info="Cíl dosažen (překážka v cílovém bodě)",
                            target_heading_deg=default_target_heading_deg,
                            speed_cap_cms=0.0,
                            lateral_offset_m=0.0,
                            path_found=True,
                            goal_reached_near_obstacle=True,
                        )

                    self._swept_frozen_at_ts = now_ts
                    virt_zones = []
                    if self._avoid_active_obs_enu is not None:
                        virt_zones.append((self._avoid_active_obs_enu[0], self._avoid_active_obs_enu[1], LIVE_OBS_HARD_RADIUS_M))

                    ok_detour, _, _ = self.plan_corridor_path(
                        e_rrp, n_rrp, heading_deg, s_robot, d_lat_robot, smp_idx,
                        traversability_active, get_traversability_cost_fn,
                        live_obs_enu, virt_zones, allow_3d_detour=True,
                    )
                    if ok_detour:
                        self.nav_state = "AVOIDING"
                    else:
                        self.nav_state = "NO_PATH"
                        self._no_path_clear_since_ts = None

        # 4. FOLLOWING (adaptivní sledování chodce jdoucího stejným směrem)
        if self.nav_state == "FOLLOWING":
            if blocking_cl is None:
                if self._clear_since_ts is None:
                    self._clear_since_ts = now_ts
                elif (now_ts - self._clear_since_ts) >= EARLY_CLEAR_HOLD_SEC:
                    self.nav_state = "CRUISING"
                    self._clear_since_ts = None
            else:
                self._clear_since_ts = None
                self._ped_last_s = blocking_cl.s_center
                self._ped_last_d_lat = blocking_cl.d_lat_center
                if v_par > 0.05:
                    self._ped_v_parallel = 0.7 * self._ped_v_parallel + 0.3 * v_par if self._ped_v_parallel > 0 else v_par
                else:
                    self._ped_v_parallel = v_par
                self._ped_last_seen_ts = now_ts

                # Pokud se chodec zastavil před robotem -> přejdeme do standardního vyčkávání
                if v_par < 0.18 and blocking_cl.min_dist_m <= (WAIT_STOP_DIST_M + 0.20):
                    self.nav_state = "WAITING_CLEARANCE"
                    self._wait_start_ts = now_ts
                    self._wait_deadline_ts = now_ts + BASE_WAIT_SEC
                    self._is_dynamic_obs = True
                    self._last_motion_ref_enu = (blocking_cl.centroid_e, blocking_cl.centroid_n)
                    self.swept_cells.clear()
                    self._record_swept_footprint(blocking_cl.points_enu, now_ts)
                # Po 2.5 s sledování ověříme podmínku předjetí (rozdíl rychlostí >= 0.6 m/s + volný levý pruh)
                elif (now_ts - self._follow_start_ts) >= FOLLOW_OBSERVE_SEC:
                    speed_diff_ms = max_speed_ms - max(0.0, self._ped_v_parallel)
                    if speed_diff_ms >= OVERTAKE_MIN_SPEED_DIFF_MS and lat_span <= FOLLOW_MAX_LAT_SPAN_M:
                        ok_ovt, _, _ = self.plan_corridor_path(
                            e_rrp, n_rrp, heading_deg, s_robot, d_lat_robot, smp_idx,
                            traversability_active, get_traversability_cost_fn,
                            live_obs_enu, [], allow_3d_detour=True,
                            preferred_min_sep_from_ped=(self._ped_last_s, self._ped_last_d_lat),
                        )
                        if ok_ovt:
                            self.nav_state = "OVERTAKING"
                            self._overtake_start_ts = now_ts

        # 5. OVERTAKING (předjíždění chodce plnou rychlostí s predikcí jeho polohy ve slepém úhlu)
        virtual_zones: List[Tuple[float, float, float]] = []
        ped_sep_constraint: Optional[Tuple[float, float]] = None

        if self.nav_state == "OVERTAKING":
            # Aktualizujeme viděného chodce v pravém pruhu, pokud je ještě v zorném poli LiDARu
            for cl in clusters:
                if abs(cl.d_lat_center - self._ped_last_d_lat) <= 0.65 and cl.s_center >= (s_robot - 0.1):
                    dt_seen = max(0.05, now_ts - self._ped_last_seen_ts)
                    if abs(cl.s_center - (self._ped_last_s + self._ped_v_parallel * dt_seen)) <= 1.2:
                        self._ped_last_s = cl.s_center
                        self._ped_last_d_lat = cl.d_lat_center
                        self._ped_last_seen_ts = now_ts
                        break

            # Predikovaná poloha chodce (i když už zmizel ve slepém úhlu za předním LiDARem!)
            dt_blind = max(0.0, now_ts - self._ped_last_seen_ts)
            s_ped_pred = self._ped_last_s + max(0.20, self._ped_v_parallel) * dt_blind

            # Blokujeme pravou stopu od predikované polohy chodce až +2.2 m před něj (anti cut-in)
            already_shifted = abs(d_lat_robot - self._ped_last_d_lat) >= 0.70
            for ds_guard in (0.0, 0.7, 1.4, OVERTAKE_MERGE_AHEAD_M):
                sg = min(self.corridor.total_length_m, s_ped_pred + ds_guard)
                if not already_shifted and sg < (s_robot + 1.10):
                    continue
                smp_g = self.corridor.sample_at_s(sg, smp_idx)
                ge = smp_g.e + self._ped_last_d_lat * smp_g.nr_e
                gn = smp_g.n + self._ped_last_d_lat * smp_g.nr_n
                virtual_zones.append((ge, gn, 0.55))

            ped_sep_constraint = (s_ped_pred, self._ped_last_d_lat)

            # Podmínka dokončení předjíždění: náskok >= 2.2 m před predikovanou polohou chodce a návrat ke stopě
            if (
                (s_robot >= (s_ped_pred + OVERTAKE_MERGE_AHEAD_M) and abs(d_lat_robot) <= 0.30)
                or (now_ts - self._overtake_start_ts) >= OVERTAKE_MAX_DURATION_SEC
            ):
                if blocking_cl is None:
                    self.nav_state = "CRUISING"
                    virtual_zones.clear()
                    ped_sep_constraint = None

        # 6. AVOIDING (objíždění statické / usazené překážky max rychlostí 1.0 m/s)
        if self.nav_state == "AVOIDING":
            # Udržujeme ochranný bod u míjené překážky, dokud ji celý podvozek nemine o >= 1.6 m
            if s_robot < (self._avoid_target_s + STATIC_PASS_CLEAR_M) and self._avoid_active_obs_enu is not None:
                virtual_zones.append((self._avoid_active_obs_enu[0], self._avoid_active_obs_enu[1], LIVE_OBS_HARD_RADIUS_M))
            elif blocking_cl is None and abs(d_lat_robot) <= 0.25:
                self.nav_state = "CRUISING"
                self._avoid_active_obs_enu = None

        # ------------------------------------------------------------------
        # VÝPOČET TRAJEKTORIE V KORIDORU A RYCHLOSTNÍCH LIMITŮ DLE STAVU
        # ------------------------------------------------------------------
        allow_3d_detour = self.nav_state in ("AVOIDING", "OVERTAKING", "NO_PATH")

        # Ve stavu CRUISING / FOLLOWING povolíme plynulé vyhýbání terénu (LIDAR-TRAVERSABILITY),
        # ale živé 3D překážky v CRUISING spouští nejprve brzdění/sledování, nikoliv okamžité kličkování.
        planner_obs_enu = live_obs_enu if allow_3d_detour else []

        path_ok, path_pts, target_d_lat = self.plan_corridor_path(
            e_rrp=e_rrp,
            n_rrp=n_rrp,
            heading_deg=heading_deg,
            s_robot=s_robot,
            d_lat_robot=d_lat_robot,
            start_sample_idx=smp_idx,
            traversability_active=traversability_active,
            get_traversability_cost_fn=get_traversability_cost_fn,
            live_obs_points_enu=planner_obs_enu,
            virtual_hard_zones_enu=virtual_zones,
            allow_3d_detour=True if self.nav_state != "OBSTACLE" else False,
            preferred_min_sep_from_ped=ped_sep_constraint,
        )

        # Ošetření ztráty průjezdné cesty během AVOIDING / OVERTAKING / CRUISING (např. obrubník/zeď přes celý koridor)
        if not path_ok and self.nav_state in ("AVOIDING", "OVERTAKING", "CRUISING"):
            if self.nav_state in ("AVOIDING", "OVERTAKING"):
                self.nav_state = "NO_PATH"
                self._no_path_clear_since_ts = None
            elif traversability_active and blocking_cl is None:
                self.nav_state = "NO_PATH"
                self._no_path_clear_since_ts = None

        # 7. NO_PATH (není průjezdná cesta -> čeká na uvolnění koridoru s hysterezí 1.0 s)
        if self.nav_state == "NO_PATH":
            if rem_goal_m <= GOAL_OCCUPIED_RADIUS_M:
                return AvoidanceDecision(
                    nav_state="CRUISING",
                    nav_info="Cíl dosažen (překážka v cílovém bodě)",
                    target_heading_deg=default_target_heading_deg,
                    speed_cap_cms=0.0,
                    lateral_offset_m=0.0,
                    path_found=True,
                    goal_reached_near_obstacle=True,
                )

            ok_check, path_pts_chk, target_d_chk = self.plan_corridor_path(
                e_rrp=e_rrp,
                n_rrp=n_rrp,
                heading_deg=heading_deg,
                s_robot=s_robot,
                d_lat_robot=d_lat_robot,
                start_sample_idx=smp_idx,
                traversability_active=traversability_active,
                get_traversability_cost_fn=get_traversability_cost_fn,
                live_obs_points_enu=live_obs_enu,
                virtual_hard_zones_enu=[],
                allow_3d_detour=True,
            )
            if ok_check:
                if self._no_path_clear_since_ts is None:
                    self._no_path_clear_since_ts = now_ts
                elif (now_ts - self._no_path_clear_since_ts) >= NO_PATH_CLEAR_HOLD_SEC:
                    self._no_path_clear_since_ts = None
                    path_ok, path_pts, target_d_lat = ok_check, path_pts_chk, target_d_chk
                    if blocking_cl is None and abs(target_d_lat) <= 0.25:
                        self.nav_state = "CRUISING"
                    else:
                        self.nav_state = "AVOIDING"
                        if blocking_cl is not None:
                            self._avoid_target_s = blocking_cl.s_max
                            self._avoid_active_obs_enu = (blocking_cl.centroid_e, blocking_cl.centroid_n)
            else:
                self._no_path_clear_since_ts = None

        # ------------------------------------------------------------------
        # SESTAVENÍ VÝSTUPNÍHO ROZHODNUTÍ (RYCHLOST, SMĚR, POPIS PRO TERMINÁL)
        # ------------------------------------------------------------------
        lookahead_m = 1.35 if (self.nav_state in ("AVOIDING", "OVERTAKING") or abs(target_d_lat - d_lat_robot) > 0.25) else 1.85
        planned_heading = (
            self._compute_lookahead_heading(e_rrp, n_rrp, s_robot, path_pts, lookahead_m, default_target_heading_deg)
            if path_ok and path_pts
            else default_target_heading_deg
        )

        # Pokud jsme v klidném CRUISING bez odchylky koridoru, zachováme přesný azimut NearWaypointu
        if self.nav_state == "CRUISING" and abs(target_d_lat) < 0.08 and abs(d_lat_robot) < 0.15:
            planned_heading = default_target_heading_deg

        speed_cap_cms = max_speed_cms

        if self.nav_state == "WAITING_CLEARANCE":
            speed_cap_cms = 0.0
            elapsed_w = max(0.0, now_ts - self._wait_start_ts)
            total_w = max(BASE_WAIT_SEC, self._wait_deadline_ts - self._wait_start_ts)
            kind_str = "pohybující se" if self._is_dynamic_obs else "statická"
            self.nav_info = f"Čeká na uvolnění ({kind_str}, {elapsed_w:.1f}/{total_w:.0f} s)"

        elif self.nav_state == "NO_PATH":
            speed_cap_cms = 0.0
            self.nav_info = "Není průjezdná cesta – čekám na uvolnění koridoru"

        elif self.nav_state == "OBSTACLE":
            dist_m = blocking_cl.min_dist_m if blocking_cl is not None else WAIT_STOP_DIST_M
            if dist_m <= WAIT_STOP_DIST_M:
                speed_cap_cms = 0.0
            else:
                ratio = max(0.0, min(1.0, (dist_m - WAIT_STOP_DIST_M) / (APPROACH_SLOW_DIST_M - WAIT_STOP_DIST_M)))
                speed_cap_cms = max_speed_cms * ratio
            self.nav_info = f"Překážka na trase ({dist_m:.1f} m) – zpomalování"

        elif self.nav_state == "FOLLOWING":
            dist_m = blocking_cl.min_dist_m if blocking_cl is not None else FOLLOW_TARGET_GAP_M
            v_cmd_ms = max(0.0, min(max_speed_ms, self._ped_v_parallel + 0.6 * (dist_m - FOLLOW_TARGET_GAP_M)))
            if dist_m <= EMERGENCY_STOP_DIST_M + 0.20:
                v_cmd_ms = 0.0
            speed_cap_cms = v_cmd_ms * 100.0
            self.nav_info = f"Sleduji chodce ({self._ped_v_parallel:.1f} m/s, odstup {dist_m:.1f} m)"

        elif self.nav_state == "AVOIDING":
            speed_cap_cms = min(max_speed_cms, AVOID_MAX_SPEED_CMS)
            self.nav_info = f"Objíždím překážku {self._format_side_offset(target_d_lat)}"

        elif self.nav_state == "OVERTAKING":
            speed_cap_cms = max_speed_cms
            self.nav_info = f"Předjíždím chodce {self._format_side_offset(target_d_lat)}"

        else:  # CRUISING
            speed_cap_cms = max_speed_cms
            if abs(target_d_lat) >= 0.20:
                self.nav_info = f"Jízda v koridoru – korekce povrchu {self._format_side_offset(target_d_lat)}"
            else:
                self.nav_info = "Jízda podle trasy"

        return AvoidanceDecision(
            nav_state=self.nav_state,
            nav_info=self.nav_info,
            target_heading_deg=planned_heading,
            speed_cap_cms=speed_cap_cms,
            lateral_offset_m=target_d_lat,
            path_found=(self.nav_state != "NO_PATH"),
            goal_reached_near_obstacle=False,
        )
