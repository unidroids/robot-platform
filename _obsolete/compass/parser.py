# parser.py
from enum import Enum
from typing import Tuple, Optional

class ParseResult(Enum):
    PROCESSING = 0
    MESSAGE = 1
    CHECKSUM_ERROR = 2
    CORRUPTED = 3

class CompassParser:
    """
    Parser for the 10-axis IMU protocol.
    Format: 0x55, TYPE, DATAL1, DATAH1, DATAL2, DATAH2, DATAL3, DATAH3, DATAL4, DATAH4, SUMCRC
    Total length is 11 bytes.
    SUMCRC is the lower 8 bits of the sum of the first 10 bytes.
    """
    
    S_WAIT_HEADER = 0
    S_WAIT_DATA = 1

    def __init__(self):
        self.state = self.S_WAIT_HEADER
        self.buffer = bytearray()
        
    def feed(self, b: int) -> Tuple[ParseResult, Optional[bytes]]:
        """
        Feeds one byte to the parser.
        Returns a tuple of (ParseResult, parsed_message_bytes).
        If ParseResult is MESSAGE, the second element is the complete 11-byte frame.
        """
        if self.state == self.S_WAIT_HEADER:
            if b == 0x55:
                self.buffer.clear()
                self.buffer.append(b)
                self.state = self.S_WAIT_DATA
            return ParseResult.PROCESSING, None

        elif self.state == self.S_WAIT_DATA:
            self.buffer.append(b)
            if len(self.buffer) == 11:
                # We have a full packet, verify checksum
                checksum = sum(self.buffer[:10]) & 0xFF
                if checksum == self.buffer[10]:
                    msg = bytes(self.buffer)
                    self.buffer.clear()
                    self.state = self.S_WAIT_HEADER
                    return ParseResult.MESSAGE, msg
                else:
                    self.buffer.clear()
                    self.state = self.S_WAIT_HEADER
                    # We might have missed another 0x55 in the corrupted data,
                    # but simple recovery is just going back to WAIT_HEADER
                    return ParseResult.CHECKSUM_ERROR, None
            
            return ParseResult.PROCESSING, None
        
        return ParseResult.PROCESSING, None
