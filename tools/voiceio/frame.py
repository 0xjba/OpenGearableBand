"""Uplink wire format: [seq16 LE][ts32 LE][LC3 payload]. ts32 = cumulative 16 kHz
mic-sample count (increments 320 per frame = 2 LC3 frames)."""
import struct
from dataclasses import dataclass

HDR = 6  # seq16 (2) + ts32 (4)

@dataclass
class UplinkFrame:
    seq: int
    ts32: int
    lc3: bytes

def parse_uplink(pkt: bytes):
    if len(pkt) < HDR:
        return None
    seq, ts32 = struct.unpack_from("<HI", pkt, 0)
    return UplinkFrame(seq=seq, ts32=ts32, lc3=pkt[HDR:])
