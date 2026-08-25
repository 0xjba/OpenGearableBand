import asyncio, sys
from bleak import BleakScanner, BleakClient

TXT_UUID = "e9a10002-4b2c-4d3e-9f5a-0123456789ab"
NAME = "gband-OLED"

async def main():
    print("scanning for", NAME, "...")
    dev = await BleakScanner.find_device_by_name(NAME, timeout=10.0)
    if not dev:
        print("NOT FOUND"); return 2
    print("found:", dev.address, "connecting...")
    async with BleakClient(dev) as c:
        print("connected:", c.is_connected)
        msg = b"hello\nfrom Mac"
        await c.write_gatt_char(TXT_UUID, msg, response=True)
        print("wrote:", msg)
        await asyncio.sleep(1.0)
        msg2 = b"M2 works!"
        await c.write_gatt_char(TXT_UUID, msg2, response=True)
        print("wrote:", msg2)
        await asyncio.sleep(0.5)
    print("done")
    return 0

sys.exit(asyncio.run(main()))
