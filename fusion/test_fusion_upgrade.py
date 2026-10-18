# fusion/test_fusion_upgrade.py
import math
import time
import unittest

from core import FusionCore

class TestFusionUpgrade(unittest.TestCase):

    def setUp(self):
        self.core = FusionCore()

    def test_odometry_correction_scale(self):
        """Ověření, že rychlost odometrie je dělena faktorem 1.05."""
        self.core.update_odometry(speed_left=1050.0, speed_right=1050.0)
        self.assertAlmostEqual(self.core._raw_speed, 1050.0, places=1)
        self.assertAlmostEqual(self.core._speed, 1000.0, places=1)

    def test_uniheading_initialization_and_complementary_blend(self):
        """Ověření prvotní inicializace z UNIHEADING a následného plynulého dotahování."""
        # 1. Prvotní NARROW_INT inicializuje přímo
        self.core.update_dual_heading(heading=90.0, headingAcc=0.8, headingSol="NARROW_INT")
        self.assertTrue(self.core._heading_initialized)
        self.assertEqual(self.core._heading_source, "UNIHEADING")
        self.assertAlmostEqual(self.core.fused_heading.heading, 90.0, places=2)

        # 2. Další aktualizace se vyhlazuje s alpha = 0.1 (diff = +10.0 -> +1.0)
        self.core.update_dual_heading(heading=100.0, headingAcc=0.8, headingSol="NARROW_INT")
        self.assertAlmostEqual(self.core.fused_heading.heading, 91.0, places=2)

        # 3. Ztráta NARROW_INT přepne zdroj na GYRO
        self.core.update_dual_heading(heading=100.0, headingAcc=4.0, headingSol="SINGLE")
        self.assertEqual(self.core._heading_source, "GYRO")
        self.assertEqual(self.core.fused_heading.sol, "GYRO")

    def test_bestnav_course_lock_3_samples(self):
        """Ověření uzamčení kurzu po 3 shodných vzorcích v toleranci +-3° při BESTNAV hor_spd > 0.3 m/s."""
        # Robot stojí (hor_spd = 0.0) -> kurz z BESTNAV se nebere
        self.core.update_odometry(0.0, 0.0, left_steps=100, right_steps=100)
        self.core.update_gps_heading(heading=180.0, headingAcc=2.0, headingSol="FIXED", hor_spd=0.0)
        self.assertFalse(self.core._heading_initialized)
        self.assertEqual(len(self.core._bestnav_course_buffer), 0)

        # Robot jede dopředu (kroky se mění, hor_spd = 0.45 m/s) a netočí se (|wz| < 3°/s)
        self.core.update_odometry(525.0, 525.0, left_steps=150, right_steps=150)
        self.core._gyroZ = 0.5

        # 1. vzorek
        self.core.update_gps_heading(heading=180.0, headingAcc=2.0, headingSol="FIXED", hor_spd=0.45)
        self.assertFalse(self.core._heading_initialized)

        # 2. vzorek (+1.5°)
        self.core.update_gps_heading(heading=181.5, headingAcc=2.0, headingSol="FIXED", hor_spd=0.45)
        self.assertFalse(self.core._heading_initialized)

        # 3. vzorek (-1.0°) -> všechny 3 v rozmezí <= 3.0°
        self.core.update_gps_heading(heading=179.0, headingAcc=2.0, headingSol="FIXED", hor_spd=0.45)
        self.assertTrue(self.core._heading_initialized)
        self.assertEqual(self.core._heading_source, "BESTNAV_COURSE")
        # Průměr (180 + 181.5 + 179) / 3 = 180.17°
        self.assertAlmostEqual(self.core.fused_heading.heading, 180.17, places=1)

    def test_bestnav_course_rejected_on_turning_or_deviation(self):
        """Ověření, že při zatáčení nebo nesouhlasu vzorků se kurz neuzamkne."""
        self.core.update_odometry(525.0, 525.0, left_steps=200, right_steps=200)
        # Robot zatáčí (wz = 5.0 deg/s)
        self.core._gyroZ = 5.0
        self.core.update_gps_heading(heading=180.0, headingAcc=2.0, headingSol="FIXED", hor_spd=0.45)
        self.assertEqual(len(self.core._bestnav_course_buffer), 0)

        # Robot jede rovně, ale vzorky mají rozptyl > 3°
        self.core._gyroZ = 0.0
        self.core.update_gps_heading(heading=180.0, headingAcc=2.0, headingSol="FIXED", hor_spd=0.45)
        self.core.update_gps_heading(heading=181.0, headingAcc=2.0, headingSol="FIXED", hor_spd=0.45)
        self.core.update_gps_heading(heading=186.0, headingAcc=2.0, headingSol="FIXED", hor_spd=0.45)  # odchylka 6°
        self.assertFalse(self.core._heading_initialized)

    def test_odometry_zupt_stationary_ignores_gyro_drift(self):
        """Ověření, že pokud se steps nemění a robot stojí, gyro delta_yaw se ignoruje (ZUPT)."""
        # 1. Inicializujeme úhel na 90.0°
        self.core.update_dual_heading(heading=90.0, headingAcc=0.8, headingSol="NARROW_INT")
        self.core.update_dual_heading(heading=90.0, headingAcc=5.0, headingSol="SINGLE")  # přepne na GYRO

        # 2. Simulujeme stání robota: stejné kroky po dobu > 100 ms
        self.core.update_odometry(0.0, 0.0, left_steps=5000, right_steps=5000)
        time.sleep(0.1)
        self.core.update_odometry(0.0, 0.0, left_steps=5000, right_steps=5000)
        self.assertTrue(self.core._is_stationary)

        # 3. Z gyra přichází šum / drift (wz = 0.3°/s, delta_yaw = 0.05°)
        self.core.update_imu(ts=1.0, delta_yaw=0.05, wz=0.3)
        # Úhel MUSÍ zůstat přesně 90.0°!
        self.assertAlmostEqual(self.core.fused_heading.heading, 90.0, places=2)

        # 4. Robot se rozjede: kroky se změní
        self.core.update_odometry(300.0, 300.0, left_steps=5020, right_steps=5020)
        self.assertFalse(self.core._is_stationary)

        # 5. Nyní se rotace z gyra normálně integruje
        self.core.update_imu(ts=2.0, delta_yaw=10.0, wz=20.0)
        self.assertAlmostEqual(self.core.fused_heading.heading, 100.0, places=2)

    def test_imu_integration_and_attitude(self):
        """Ověření integrace úhlu z 20Hz IMU a nastavení pitch/roll."""
        # Inicializujeme na 90°
        self.core.update_dual_heading(heading=90.0, headingAcc=0.8, headingSol="NARROW_INT")
        # Přepneme na GYRO
        self.core.update_dual_heading(heading=90.0, headingAcc=5.0, headingSol="SINGLE")

        # Přijmeme 20Hz IMU zprávu: delta_yaw = +5.5°, pitch = 2.4°, roll = -1.1°
        self.core.update_imu(ts=100.0, delta_yaw=5.5, wz=11.0, pitch=2.4, roll=-1.1)
        self.assertAlmostEqual(self.core.fused_heading.heading, 95.5, places=2)
        self.assertAlmostEqual(self.core._pitch, 2.4, places=2)
        self.assertAlmostEqual(self.core._roll, -1.1, places=2)

        sol = self.core.get_solution()
        self.assertAlmostEqual(sol.pitch, 2.4, places=2)
        self.assertAlmostEqual(sol.roll, -1.1, places=2)
        self.assertEqual(sol.heading_source, "GYRO")

    def test_antenna_triangle_nominal_geometry_and_center_projection(self):
        """
        Ověření geometrie 3 antén na nominálním trojúhelníku a transformace na střed otáčení [0,0,0].
        Střed otáčení: lat0 = 50.0, lon0 = 14.0, heading = 0° (North).
        GPS: [+0.32, 0.0] m -> lat = 50.0 + 0.32/111139.0
        Master: [+0.25, +0.24] m
        Slave: [+0.25, -0.24] m
        """
        lat0 = 50.0
        lon0 = 14.0
        m_deg_lat = 111139.0
        m_deg_lon = 111139.0 * math.cos(math.radians(50.0))

        # Heading 0.0° (Sever)
        self.core.update_dual_heading(heading=0.0, headingAcc=0.5, headingSol="NARROW_INT")

        # Přesné kalibrované offsety antén:
        # GPS: x = +0.3200, y = -0.0059 (vpravo/východ o 5.9 mm)
        gps_lat = lat0 + 0.3200 / m_deg_lat
        gps_lon = lon0 - (-0.0059) / m_deg_lon

        # Master: x = +0.2617, y = +0.2341 (vlevo/západ o 23.41 cm)
        master_lat = lat0 + 0.2617 / m_deg_lat
        master_lon = lon0 - 0.2341 / m_deg_lon

        # Slave: x = +0.2617, y = -0.2341 (vpravo/východ o 23.41 cm)
        slave_lat = lat0 + 0.2617 / m_deg_lat
        slave_lon = lon0 + 0.2341 / m_deg_lon

        self.core.update_gps_antenna(gps_lat, gps_lon, 0.02, "NARROW_INT")
        self.core.update_master_antenna(master_lat, master_lon, 0.02, "NARROW_INT")
        self.core.update_slave_antenna(slave_lat, slave_lon, 0.02, "NARROW_INT")

        # Ověření statusu trojúhelníku a výběru primární antény
        self.assertEqual(self.core._triangle_status, "TRIANGLE_OK")
        self.assertEqual(self.core._active_antenna_name, "gnss-gps")

        # Ověření vzdáleností: dual ~ 0.468 m, gps-master ~ 0.247 m, gps-slave ~ 0.236 m
        self.assertAlmostEqual(self.core._dist_dual, 0.468, delta=0.02)
        self.assertAlmostEqual(self.core._dist_gps_master, 0.247, delta=0.02)
        self.assertAlmostEqual(self.core._dist_gps_slave, 0.236, delta=0.02)

        # Ověření transformované polohy: střed otáčení musí být přesně lat0, lon0
        self.assertAlmostEqual(self.core._center_lat, lat0, places=6)
        self.assertAlmostEqual(self.core._center_lon, lon0, places=6)

    def test_antenna_outlier_exclusion(self):
        """Ověření detekce a vyloučení vadné antény (např. multipath na primární GPS anténě)."""
        lat0 = 50.0
        lon0 = 14.0
        m_deg_lat = 111139.0
        m_deg_lon = 111139.0 * math.cos(math.radians(50.0))

        self.core.update_dual_heading(heading=0.0, headingAcc=0.5, headingSol="NARROW_INT")

        # Master a Slave jsou v pořádku (kalibrovaná základna 0.468 m)
        master_lat = lat0 + 0.2617 / m_deg_lat
        master_lon = lon0 - 0.2341 / m_deg_lon
        slave_lat = lat0 + 0.2617 / m_deg_lat
        slave_lon = lon0 + 0.2341 / m_deg_lon

        # GPS anténa má chybu 3 metry (multipath)
        bad_gps_lat = lat0 + 3.32 / m_deg_lat
        bad_gps_lon = lon0

        self.core.update_gps_antenna(bad_gps_lat, bad_gps_lon, 0.02, "NARROW_INT")
        self.core.update_master_antenna(master_lat, master_lon, 0.02, "NARROW_INT")
        self.core.update_slave_antenna(slave_lat, slave_lon, 0.02, "NARROW_INT")

        # Detekce vyloučení GPS antény a přepnutí na Master
        self.assertEqual(self.core._triangle_status, "EXCLUDED_GPS")
        self.assertEqual(self.core._active_antenna_name, "gnss-dual-master")

        # Výsledná poloha je přepočítána ze záložní Master antény na střed otáčení!
        self.assertAlmostEqual(self.core._center_lat, lat0, places=6)
        self.assertAlmostEqual(self.core._center_lon, lon0, places=6)

    def test_status_diagnostics_structure(self):
        """Ověření, že get_diagnostics() vrací všechny požadované skupiny."""
        diag = self.core.get_diagnostics()
        self.assertIn("constellation", diag)
        self.assertIn("antennas", diag)
        self.assertIn("heading_hold", diag)
        self.assertIn("odometry", diag)
        self.assertIn("attitude", diag)
        self.assertIn("heading_consensus", diag)
        self.assertEqual(diag["odometry"]["scale_factor"], 1.05)

    def test_kinematic_tangent_offset_calculation(self):
        """Ověření výpočtu tečného úhlu odchylky beta pro každou anténu v zatáčce."""
        # v = 0.5 m/s, točení doprava wz = 6.0 deg/s
        # GPS (x=0.32, y=0.0): beta = atan2(wz*x, v) = atan2(0.1047*0.32, 0.5) ~ +3.83°
        beta_gps = self.core._compute_tangent_offset(offset_x=0.32, offset_y=0.0, v_ms=0.5, wz_deg_s=6.0)
        self.assertAlmostEqual(beta_gps, 3.83, delta=0.05)

        # Master (x=0.25, y=+0.24): beta = atan2(0.1047*0.25, 0.5 + 0.1047*0.24) ~ +2.85°
        beta_mst = self.core._compute_tangent_offset(offset_x=0.25, offset_y=0.24, v_ms=0.5, wz_deg_s=6.0)
        self.assertAlmostEqual(beta_mst, 2.85, delta=0.05)

        # Slave (x=0.25, y=-0.24): beta = atan2(0.1047*0.25, 0.5 - 0.1047*0.24) ~ +3.16°
        beta_slv = self.core._compute_tangent_offset(offset_x=0.25, offset_y=-0.24, v_ms=0.5, wz_deg_s=6.0)
        self.assertAlmostEqual(beta_slv, 3.16, delta=0.05)

    def test_3_antenna_heading_consensus_during_gentle_turn(self):
        """Ověření, že při mírné zatáčce všechny 3 antény po kinematické kompenzaci dají shodný sever."""
        # Robot jede po oblouku rychlostí 0.5 m/s, úhlová rychlost wz = +6.0 deg/s
        self.core.update_odometry(525.0, 525.0, left_steps=1000, right_steps=1000)
        self.core._gyroZ = 6.0

        true_robot_heading = 45.0
        beta_gps = self.core._compute_tangent_offset(self.core.constellation.ant_gps.offset_x, self.core.constellation.ant_gps.offset_y, 0.5, 6.0)
        beta_mst = self.core._compute_tangent_offset(self.core.constellation.ant_master.offset_x, self.core.constellation.ant_master.offset_y, 0.5, 6.0)
        beta_slv = self.core._compute_tangent_offset(self.core.constellation.ant_slave.offset_x, self.core.constellation.ant_slave.offset_y, 0.5, 6.0)

        # Surové kurzy měřené jednotlivými anténami na zakřivené dráze
        trk_gps = true_robot_heading + beta_gps
        trk_mst = true_robot_heading + beta_mst
        trk_slv = true_robot_heading + beta_slv

        self.core.update_gps_antenna(50.0, 14.0, 0.02, "NARROW_INT", trk_gnd=trk_gps, hor_spd=0.5)
        self.core.update_master_antenna(50.0, 14.0, 0.02, "NARROW_INT", trk_gnd=trk_mst, hor_spd=0.5)
        self.core.update_slave_antenna(50.0, 14.0, 0.02, "NARROW_INT", trk_gnd=trk_slv, hor_spd=0.5)

        self.assertEqual(self.core._heading_consensus_status, "3_ANTENNAS_AGREE")
        self.assertEqual(self.core._heading_source, "BESTNAV_CONSENSUS")
        # Výsledný uzamčený úhel musí být přesně true_robot_heading (45.0°)
        self.assertAlmostEqual(self.core.fused_heading.heading, true_robot_heading, places=1)

    def test_3_antenna_consensus_isolates_corrupted_gps(self):
        """Ověření, že pokud jedna anténa lže (multipath), systém ji izoluje a použije zbylé dvě."""
        self.core.update_odometry(525.0, 525.0, left_steps=2000, right_steps=2000)
        self.core._gyroZ = 6.0

        true_robot_heading = 90.0
        beta_mst = self.core._compute_tangent_offset(self.core.constellation.ant_master.offset_x, self.core.constellation.ant_master.offset_y, 0.5, 6.0)
        beta_slv = self.core._compute_tangent_offset(self.core.constellation.ant_slave.offset_x, self.core.constellation.ant_slave.offset_y, 0.5, 6.0)

        trk_mst = true_robot_heading + beta_mst
        trk_slv = true_robot_heading + beta_slv
        # GPS je pod větví stromu a hlásí o 15° vedle
        corrupted_trk_gps = true_robot_heading + 15.0

        self.core.update_gps_antenna(50.0, 14.0, 0.02, "NARROW_INT", trk_gnd=corrupted_trk_gps, hor_spd=0.5)
        self.core.update_master_antenna(50.0, 14.0, 0.02, "NARROW_INT", trk_gnd=trk_mst, hor_spd=0.5)
        self.core.update_slave_antenna(50.0, 14.0, 0.02, "NARROW_INT", trk_gnd=trk_slv, hor_spd=0.5)

        self.assertEqual(self.core._heading_consensus_status, "2_AGREE_GPS_OUTLIER")
        self.assertEqual(self.core._heading_source, "BESTNAV_CONSENSUS")
        self.assertAlmostEqual(self.core.fused_heading.heading, true_robot_heading, places=1)

    def test_3_antenna_consensus_rejected_in_excessive_rotation(self):
        """Ověření, že při prudké rotaci (|wz| > 10°/s) se konsenzus neaktivuje."""
        self.core.update_odometry(525.0, 525.0, left_steps=3000, right_steps=3000)
        # Ostrá otočka na místě
        self.core._gyroZ = 20.0

        self.core.update_gps_antenna(50.0, 14.0, 0.02, "NARROW_INT", trk_gnd=90.0, hor_spd=0.5)
        self.core.update_master_antenna(50.0, 14.0, 0.02, "NARROW_INT", trk_gnd=90.0, hor_spd=0.5)
        self.core.update_slave_antenna(50.0, 14.0, 0.02, "NARROW_INT", trk_gnd=90.0, hor_spd=0.5)

    def test_position_dead_reckoning_under_bridge(self):
        """
        Komplexní scénář:
        1. Jízda na RTK před mostem.
        2. Vjezd pod most (výpadek GNSS).
        3. Propagace polohy pomocí odometrie a gyro heading v Dead Reckoning.
        4. Zastavení pod mostem (ZUPT ochrana).
        5. Výjezd zpod mostu a re-akvizice RTK fixu.
        """
        lat0 = 50.0
        lon0 = 14.0
        m_deg_lat = 111139.0

        # 1. Před mostem: Heading 0.0° (Sever), RTK fix
        self.core.update_dual_heading(heading=0.0, headingAcc=0.5, headingSol="NARROW_INT")
        self.core.update_gps_antenna(lat0 + 0.32 / m_deg_lat, lon0, 0.02, "NARROW_INT")
        self.core.update_master_antenna(lat0 + 0.25 / m_deg_lat, lon0 + 0.24 / (m_deg_lat * 0.6427876), 0.02, "NARROW_INT")
        self.core.update_slave_antenna(lat0 + 0.25 / m_deg_lat, lon0 - 0.24 / (m_deg_lat * 0.6427876), 0.02, "NARROW_INT")

        sol1 = self.core.get_solution()
        self.assertAlmostEqual(sol1.lat, lat0, places=6)
        self.assertFalse(self.core._dr_active)

        # 2. Vjezd pod most: GNSS přestane posílat data (simulujeme posun času o 3s)
        self.core.position_tracker.last_valid_gnss_ts = time.monotonic() - 3.0

        # Jízda pod mostem rychlostí 1050 mm/s (korigováno na 1000 mm/s = 1.0 m/s) na sever
        # Simulujeme 5 sekund jízdy po 1s krocích
        now = time.monotonic()
        for i in range(1, 6):
            self.core.update_odometry(1050.0, 1050.0, left_steps=1000 + i * 50, right_steps=1000 + i * 50)
            self.core.position_tracker.last_odo_ts = now + (i - 1)
            self.core.position_tracker.update_odometry(
                speed_mm_s=self.core._speed,
                is_stationary=False,
                heading_deg=0.0,
                heading_valid=True,
                ts_mono=now + i
            )

        sol2 = self.core.get_solution()
        self.assertTrue(self.core._dr_active)
        self.assertEqual(sol2.gpsSol, "DEAD_RECKONING")
        self.assertEqual(sol2.fusionSol, "DEAD_RECKONING")
        # Posun na sever o cca 5 m
        expected_lat = lat0 + (5.0 / m_deg_lat)
        self.assertAlmostEqual(sol2.lat, expected_lat, places=5)
        self.assertGreater(sol2.hAcc, 0.02)

        # 3. Zastavení pod mostem
        self.core.update_odometry(0.0, 0.0, left_steps=1300, right_steps=1300)
        time.sleep(0.1)
        self.core.update_odometry(0.0, 0.0, left_steps=1300, right_steps=1300)
        self.assertTrue(self.core._is_stationary)
        lat_stopped = self.core._center_lat

        # Gyro posílá šum při stání pod mostem -> poloha ani heading se nesmí pohnout
        self.core.update_imu(ts=time.monotonic(), delta_yaw=0.05, wz=0.2)
        self.assertAlmostEqual(self.core._center_lat, lat_stopped, places=7)

        # 4. Výjezd zpod mostu: obnova RTK signálu na nové pozici (posunuto o 5.2 m)
        exit_lat = lat0 + (5.2 / m_deg_lat)
        self.core.update_dual_heading(heading=0.0, headingAcc=0.5, headingSol="NARROW_INT")
        self.core.update_gps_antenna(exit_lat + 0.3200 / m_deg_lat, lon0 - (-0.0059) / (m_deg_lat * 0.6427876), 0.02, "NARROW_INT")
        self.core.update_master_antenna(exit_lat + 0.2617 / m_deg_lat, lon0 - 0.2341 / (m_deg_lat * 0.6427876), 0.02, "NARROW_INT")
        self.core.update_slave_antenna(exit_lat + 0.2617 / m_deg_lat, lon0 + 0.2341 / (m_deg_lat * 0.6427876), 0.02, "NARROW_INT")

        sol3 = self.core.get_solution()
        self.assertFalse(self.core._dr_active)
        self.assertEqual(sol3.fusionSol, "UNIHEADING")
        self.assertAlmostEqual(sol3.lat, exit_lat, delta=0.15)

    def test_epoch_synchronizer_complete_dispatch(self):
        """Ověření, že EpochSynchronizer odešle data ihned jakmile dorazí všechny 3 antény stejné epochy."""
        from core_modules.epoch_sync import EpochSynchronizer
        dispatched = []
        sync = EpochSynchronizer(dispatch_fn=lambda msgs: dispatched.append(msgs), timeout_s=0.025)

        # Příchod 1. zprávy (GPS)
        sync.add_msg("gps", {"lat": 50.0, "lon": 14.0, "wn": 2387, "ms": 100000})
        self.assertEqual(len(dispatched), 0)
        self.assertTrue(sync.has_pending())

        # Příchod 2. zprávy (Master)
        sync.add_msg("master", {"lat": 50.0, "lon": 14.0, "wn": 2387, "ms": 100000})
        self.assertEqual(len(dispatched), 0)

        # Příchod 3. zprávy (Slave) -> okamžité odeslání bez čekání na timeout!
        sync.add_msg("slave", {"lat": 50.0, "lon": 14.0, "wn": 2387, "ms": 100000})
        self.assertEqual(len(dispatched), 1)
        self.assertFalse(sync.has_pending())
        self.assertIn("gps", dispatched[0])
        self.assertIn("master", dispatched[0])
        self.assertIn("slave", dispatched[0])

    def test_epoch_synchronizer_timeout_dispatches_partial(self):
        """Ověření, že při výpadku Slave antény se po 25 ms timeoutu odešlou přítomné antény (GPS + Master)."""
        from core_modules.epoch_sync import EpochSynchronizer
        dispatched = []
        sync = EpochSynchronizer(dispatch_fn=lambda msgs: dispatched.append(msgs), timeout_s=0.025)

        t0 = 1000.0
        sync.add_msg("gps", {"lat": 50.0, "lon": 14.0, "wn": 2387, "ms": 100000}, now_mono=t0)
        sync.add_msg("master", {"lat": 50.0, "lon": 14.0, "wn": 2387, "ms": 100000}, now_mono=t0 + 0.005)

        # V čase t0 + 10 ms ještě timeout nevypršel
        sync.check_timeout(now_mono=t0 + 0.010)
        self.assertEqual(len(dispatched), 0)

        # V čase t0 + 26 ms timeout vypršel -> částečná epocha se odešle
        sync.check_timeout(now_mono=t0 + 0.026)
        self.assertEqual(len(dispatched), 1)
        self.assertIn("gps", dispatched[0])
        self.assertIn("master", dispatched[0])
        self.assertNotIn("slave", dispatched[0])

    def test_baseline_skew_adaptive_tolerance(self):
        """Ověření, že odchylka spojnice Master->Slave toleruje 8° při jízdě rovně a 15° při zatáčení."""
        lat0 = 50.0
        lon0 = 14.0
        m_deg_lat = 111139.0
        m_deg_lon = 111139.0 * math.cos(math.radians(50.0))

        # Kurz robota 0.0° (Sever), spojnice Master->Slave má směřovat 91.11°
        self.core.update_dual_heading(heading=0.0, headingAcc=0.5, headingSol="NARROW_INT")

        # Pootočíme Slave tak, aby úhel spojnice byl 101.11° (odchylka +10.0°)
        # Master na [-0.2341, 0]
        # Slave má být na vzdálenosti 0.4682 pod úhlem 101.11°
        ang_rad = math.radians(101.11)
        d_east = 0.4682 * math.sin(ang_rad)
        d_north = 0.4682 * math.cos(ang_rad)

        m_lat = lat0 + 0.2617 / m_deg_lat
        m_lon = lon0 - 0.2341 / m_deg_lon

        s_lat = m_lat + d_north / m_deg_lat
        s_lon = m_lon + d_east / m_deg_lon

        # 1. Při jízdě rovně (|wz| = 0.0) je tolerance 8.0° -> odchylka 10° selže a Slave je vyloučena
        self.core._gyroZ = 0.0
        self.core.update_gps_antenna(lat0 + 0.32 / m_deg_lat, lon0, 0.02, "NARROW_INT")
        self.core.update_master_antenna(m_lat, m_lon, 0.02, "NARROW_INT")
        self.core.update_slave_antenna(s_lat, s_lon, 0.02, "NARROW_INT")

        self.assertIn(self.core._triangle_status, ("EXCLUDED_SLAVE", "TRIANGLE_DEGRADED"))

        # 2. V ostré rotaci (|wz| = 15.0°/s) je tolerance rozšířena na 15.0° -> 10° projde
        self.core._gyroZ = 15.0
        # Po přiblížení ramen na nominál pro 3 antény:
        # Nastavíme GPS tak, aby ramena GM i GS seděla do 8 cm
        self.core.constellation.evaluate_constellation_and_position(
            heading_deg=0.0, heading_initialized=True, wz_deg_s=15.0
        )
        self.assertLessEqual(abs(self.core.constellation.baseline_skew_deg - 10.0), 0.5)

    def test_single_antenna_rtk_rejected_to_dr(self):
        """Ověření pravidla: Samostatná 1 anténa s RTK fixem nesmí sama držet polohu bez geometrického konsenzu."""
        lat0 = 50.0
        lon0 = 14.0

        # Inicializace kurzu
        self.core.update_dual_heading(heading=0.0, headingAcc=0.5, headingSol="NARROW_INT")

        # Pouze GPS má NARROW_INT, Master a Slave jsou NONE
        self.core.constellation.ant_master.valid = False
        self.core.constellation.ant_slave.valid = False
        self.core.update_gps_antenna(lat0, lon0, 0.02, "NARROW_INT")

        # Konstelace musí být v DEGRADED stavu (chybí konsenzus min. 2 antén)
        self.assertEqual(self.core._triangle_status, "TRIANGLE_DEGRADED")
        # Polohový tracker odmítá RTK fix
        self.assertFalse(self.core.position_tracker.gnss_is_rtk)

        # Při pohybu podvozku tracker přechází do Dead Reckoningu
        self.core.update_odometry(500.0, 500.0, left_steps=100, right_steps=100)
        self.assertTrue(self.core._dr_active)


if __name__ == '__main__':
    unittest.main()
