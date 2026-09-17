import time
from imu_serial import ImuSerialIO
import builders

imu = ImuSerialIO('/dev/robot-compass', 230400)
imu.open()

while imu.get_message(timeout=0.0) is not None:
    pass

print("Sending read for 0x02 (RSW)...")
imu.send_command(builders.build_read(0x02))

st = time.time()
while time.time() - st < 1.0:
    msg = imu.get_message(timeout=0.1)
    if msg and msg[1] == 0x5F:
        print("Received 5F:", [hex(b) for b in msg])
        break

print("Sending read for 0x03 (RRATE)...")
imu.send_command(builders.build_read(0x03))
st = time.time()
while time.time() - st < 1.0:
    msg = imu.get_message(timeout=0.1)
    if msg and msg[1] == 0x5F:
        print("Received 5F:", [hex(b) for b in msg])
        break

imu.close()
