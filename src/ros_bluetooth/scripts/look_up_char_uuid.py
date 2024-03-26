from bleak import BleakClient
import asyncio

# device_address = "411B942B-E6E7-9958-6FD4-C127C275D013" 
device_address = "A0:9E:1A:D3:66:6B"  # Replace with the address of your target device

async def discover_services_and_characteristics():
    async with BleakClient(device_address) as client:
        # Ensure the device is connected
        if not await client.is_connected():
            print("Failed to connect")
            return
        
        print("Connected. Discovering services and characteristics...")
        services = await client.get_services()
        for service in services:
            print(f"Service: {service.uuid}, {service.description}")
            for char in service.characteristics:
                if "read" in char.properties:
                    try:
                        value = bytes(await client.read_gatt_char(char.uuid))
                        print(f"  Characteristic: {char.uuid}, Value: {value}")
                    except Exception as e:
                        print(f"  Characteristic: {char.uuid}, Value: Could not read")
                else:
                    print(f"  Characteristic: {char.uuid}, Properties: {char.properties}")

asyncio.run(discover_services_and_characteristics())
