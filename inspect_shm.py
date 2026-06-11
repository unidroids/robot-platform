import struct
from multiprocessing import shared_memory
import time
try:
    shm = shared_memory.SharedMemory(name='vision_shm_left')
    seq, ts = struct.unpack_from('q d', shm.buf, 0)
    print("SHM Left:", seq, ts)
    shm.close()
except Exception as e:
    print("Error:", e)
