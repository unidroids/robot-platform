# builders.py
import struct

def build_command(addr: int, data: int) -> bytes:
    """Helper to build standard write command: FF AA ADDR DATAL DATAH"""
    return bytes([0xFF, 0xAA, addr, data & 0xFF, (data >> 8) & 0xFF])

def build_unlock() -> bytes:
    """Unlock register to allow writing."""
    return build_command(0x69, 0xB588)

def build_save() -> bytes:
    """Save configuration."""
    return build_command(0x00, 0x0000)

def build_read(addr: int) -> bytes:
    """Read register."""
    return bytes([0xFF, 0xAA, 0x27, addr & 0xFF, 0x00])

def build_reboot() -> bytes:
    """Reboot (drops unsaved config, acts like cancel)."""
    return build_command(0x00, 0x00FF)

def build_factory_reset() -> bytes:
    """Factory reset."""
    return build_command(0x00, 0x0001)

def build_calibrate_compass() -> bytes:
    """Set calibration mode: Magnetic Field Calibration (Spherical Fitting)."""
    return build_command(0x01, 0x0007)

def build_calibrate_acc() -> bytes:
    """Set calibration mode: Auto add-up calibration."""
    return build_command(0x01, 0x0001)

def build_rsw(mask: int) -> bytes:
    """Set output content mask."""
    return build_command(0x02, mask)

def build_rrate(rate_code: int) -> bytes:
    """Set output rate."""
    return build_command(0x03, rate_code)

def build_baud(baud_code: int) -> bytes:
    """Set baud rate (0x02=9600, 0x06=115200 etc)."""
    return build_command(0x04, baud_code)

def build_bandwidth(bw_code: int) -> bytes:
    """Set bandwidth."""
    return build_command(0x1F, bw_code)
