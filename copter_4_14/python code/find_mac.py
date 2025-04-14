import asyncio
from bleak import BleakScanner

async def scan():
    devices = await BleakScanner.discover()
    for device in devices:
        print(f"Device found: {device.address} - {device.name}")

asyncio.run(scan())
