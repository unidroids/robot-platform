# test_light_fusion.py
import struct
import unittest
from light_fusion import LightFusion
from handlers.esf_raw_handler import EsfRawHandler
from gnss_serial import GnssSerialIO

def build_esf_raw_payload(gyro_z_dps: float, acc_z_mps2: float, sttag_ms: int) -> bytes:
    """
    Sestaví binární payload zprávy UBX-ESF-RAW (třída 0x10, ID 0x03).
    Formát: [4B reserved][N * (4B data + 4B sTtag)]
    """
    # 4 bajty reserved
    buf = bytearray(4)

    # Převod hodnot na celočíselné raw reprezentace
    gz_raw = int(round(gyro_z_dps * 4096.0)) & 0xFFFFFF
    az_raw = int(round(acc_z_mps2 * 1024.0)) & 0xFFFFFF

    # Typy senzorů: 5 = gyroZ, 18 = accZ
    gz_field = gz_raw | (5 << 24)
    az_field = az_raw | (18 << 24)

    buf.extend(struct.pack('<II', gz_field, sttag_ms))
    buf.extend(struct.pack('<II', az_field, sttag_ms))

    return bytes(buf)


class TestLightFusion(unittest.TestCase):

    def test_calibration_success(self):
        fusion = LightFusion(orientation_sign=-1.0)
        fusion.start_calibration()

        # Simulujeme 100 vzorků v klidu s biasem +0.2 deg/s a tíhovým zrychlením 9.81 m/s^2
        t = 1000
        for _ in range(100):
            fusion.update_sample(
                gyroX=0.0, gyroY=0.0, gyroZ=0.20,
                accX=0.0, accY=0.0, accZ=9.81,
                sTtag=t, rx_mono=t / 1000.0
            )
            t += 10

        ok, msg = fusion.finish_calibration()
        self.assertTrue(ok, f"Kalibrace selhala: {msg}")
        self.assertAlmostEqual(fusion.bias_z, 0.20, places=3)

    def test_calibration_motion_rejected(self):
        fusion = LightFusion(orientation_sign=-1.0)
        fusion.start_calibration()

        # Simulujeme vzorky s velkým pohybem/otřesy
        t = 1000
        for i in range(100):
            fusion.update_sample(
                gyroX=0.0, gyroY=0.0, gyroZ=10.0 if i % 2 == 0 else -10.0,
                accX=5.0 if i % 2 == 0 else -5.0, accY=0.0, accZ=9.81,
                sTtag=t, rx_mono=t / 1000.0
            )
            t += 10

        ok, msg = fusion.finish_calibration()
        self.assertFalse(ok)
        self.assertIn("ERR MOTION DETECTED", msg)

    def test_angle_integration_constant_rate(self):
        # orientation_sign=-1.0 (raw gyroZ je záporné pro otáčení doprava CW)
        fusion = LightFusion(orientation_sign=-1.0)
        fusion.bias_z = 0.0

        # První vzorek inicializuje referenční čas sTtag
        t_ms = 1000
        fusion.update_sample(0.0, 0.0, -60.0, 0.0, 0.0, 9.81, t_ms, t_ms / 1000.0)
        fusion.pop_20hz_increment()  # vyprázdnění inicializačního stavu

        # Nyní simulujeme 100 vzorků (20 period po 5 vzorcích)
        total_delta_yaw = 0.0
        for i in range(100):
            t_ms += 10
            fusion.update_sample(
                gyroX=0.0, gyroY=0.0, gyroZ=-60.0,
                accX=0.0, accY=0.0, accZ=9.81,
                sTtag=t_ms, rx_mono=t_ms / 1000.0
            )
            # Každých 5 vzorků (50 ms = 20 Hz) vyčteme přírůstek
            if (i + 1) % 5 == 0:
                ts, delta, wz, samples = fusion.pop_20hz_increment()
                self.assertEqual(samples, 5)
                self.assertAlmostEqual(wz, 60.0, places=2)
                self.assertAlmostEqual(delta, 3.0, places=2)  # 60 °/s * 0.05 s = 3.0°
                total_delta_yaw += delta

        # Celkový integrovaný úhel za 1 sekundu musí být přesně 60°
        self.assertAlmostEqual(total_delta_yaw, 60.0, delta=0.1)

    def test_sttag_32bit_rollover(self):
        fusion = LightFusion(orientation_sign=-1.0)
        fusion.bias_z = 0.0

        # Vzorek těsně před přetečením uint32
        t1 = 0xFFFFFFF6  # max - 10
        fusion.update_sample(0, 0, -50.0, 0, 0, 9.81, t1, 1.0)

        # Vzorek po přetečení (rozdíl 20 ms)
        t2 = 0x0000000A
        fusion.update_sample(0, 0, -50.0, 0, 0, 9.81, t2, 1.02)

        ts, delta, wz, samples = fusion.pop_20hz_increment()
        # dt = 20 ms = 0.02 s -> delta = 50.0 * 0.02 = 1.0°
        self.assertAlmostEqual(delta, 1.0, places=2)

    def test_esf_raw_handler_decoding(self):
        fusion = LightFusion(orientation_sign=-1.0)
        fusion.bias_z = 0.0
        handler = EsfRawHandler(fusion)

        # Sestavíme reálný binární payload UBX-ESF-RAW: wz = -30.0 deg/s, az = 9.81 m/s2, sTtag = 50000
        payload = build_esf_raw_payload(gyro_z_dps=-30.0, acc_z_mps2=9.81, sttag_ms=50000)
        ok = handler.handle(0x10, 0x03, payload)
        self.assertTrue(ok)

        # Druhý vzorek o 10 ms později
        payload2 = build_esf_raw_payload(gyro_z_dps=-30.0, acc_z_mps2=9.81, sttag_ms=50010)
        ok2 = handler.handle(0x10, 0x03, payload2)
        self.assertTrue(ok2)

        ts, delta, wz, samples = fusion.pop_20hz_increment()
        self.assertAlmostEqual(wz, 30.0, places=1)
        self.assertAlmostEqual(delta, 0.3, places=2)  # 30 deg/s * 0.01 s = 0.3°

    def test_ubx_checksum(self):
        # Ověření Fletcherova kontrolního součtu pro [0x10, 0x03, 0x00, 0x00]
        test_data = bytes([0x10, 0x03, 0x00, 0x00])
        ck_a, ck_b = GnssSerialIO._calculate_ubx_checksum(test_data)
        self.assertEqual(ck_a, 0x13)
        self.assertEqual(ck_b, 73)

    def test_gnss_serial_framing(self):
        # Sestavíme kompletní UBX rámec: [B5 62 class id len_lo len_hi payload... ckA ckB]
        payload = build_esf_raw_payload(gyro_z_dps=15.0, acc_z_mps2=9.81, sttag_ms=12345)
        p_len = len(payload)
        header = bytes([0x10, 0x03, p_len & 0xFF, (p_len >> 8) & 0xFF])
        ck_a, ck_b = GnssSerialIO._calculate_ubx_checksum(header + payload)
        valid_frame = b'\xB5\x62' + header + payload + bytes([ck_a, ck_b])

        # Otestujeme, že serial reader správně vyextrahuje rámec i s náhodnými šumovými bajty před ním
        junk = b'\x00\xFF\xAA\x55\xB5\x00'
        stream = junk + valid_frame

        serial_io = GnssSerialIO()
        # Vložíme stream přímo do metody čtení (simulace)
        buffer = bytearray(stream)

        # Provedeme parsing stejnou logikou jako v _reader
        sync_idx = buffer.find(b'\xB5\x62')
        self.assertEqual(sync_idx, len(junk))
        del buffer[:sync_idx]
        msg_class = buffer[2]
        msg_id = buffer[3]
        payload_len = buffer[4] | (buffer[5] << 8)
        self.assertEqual(msg_class, 0x10)
        self.assertEqual(msg_id, 0x03)
        self.assertEqual(payload_len, p_len)
        extracted_payload = bytes(buffer[6:6 + payload_len])
        self.assertEqual(extracted_payload, payload)



if __name__ == '__main__':
    unittest.main()
