import asyncio
from bleak import BleakScanner

async def scan_for_devices():
    devices = await BleakScanner.discover()
    for device in devices:
        print(f"Device found: {device.name}, Address: {device.address}")

asyncio.run(scan_for_devices())