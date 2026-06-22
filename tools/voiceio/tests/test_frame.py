import struct
from voiceio.frame import parse_uplink, UplinkFrame

def test_parse_uplink_extracts_seq_ts32_and_lc3():
    lc3 = bytes(range(80))
    pkt = struct.pack("<HI", 1234, 320) + lc3   # seq16, ts32, then 80B
    f = parse_uplink(pkt)
    assert isinstance(f, UplinkFrame)
    assert f.seq == 1234
    assert f.ts32 == 320
    assert f.lc3 == lc3

def test_parse_uplink_rejects_short_packet():
    assert parse_uplink(b"\x00\x00\x00") is None   # < 6-byte header
