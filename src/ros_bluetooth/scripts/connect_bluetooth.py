import asyncio
from bleak import BleakClient
from datetime import datetime  # Import the datetime module
import matplotlib.pyplot as plt

# device_address = "411B942B-E6E7-9958-6FD4-C127C275D013" 
device_address = "A0:9E:1A:D3:66:6B"  # Replace with the address of your Bluetooth device
characteristic_uuid = "00002a37-0000-1000-8000-00805f9b34fb"  # Replace with the characteristic UUID for streaming data

heart_rates = list()
timestamps = list()

# This function will be called every time a notification is received from the device
def notification_handler(sender, data):
    # Assuming 'data' is a bytearray like bytearray(b'\x00o')
    flags = data[0]

    # Check if the heart rate value format is UINT8 or UINT16
    if flags & 0x01:
        heart_rate = int.from_bytes(data[1:3], byteorder='little')
    else:
        heart_rate = data[1]  # UINT8 format, directly take the second byte as heart rate

    # Get the current time
    now = datetime.now()
    # Format the time in a human-readable format, for example, "2023-03-26 12:15:09"
    timestamp = now.strftime("%Y-%m-%d %H:%M:%S")

    print(f"{timestamp} - Received data from {sender}: Heart Rate Measurement: {heart_rate}")

    heart_rates.append(heart_rate)
    timestamps.append(timestamp)


async def main():
    async with BleakClient(device_address) as client:
        # Ensure the device is connected
        if not await client.is_connected():
            print("Failed to connect")
            return

        # Set up notification on the characteristic
        await client.start_notify(characteristic_uuid, notification_handler)

        # Keep the script running while data is being streamed.
        print("Streaming data, press Ctrl+C to exit...")
        try:
            while True:
                await asyncio.sleep(1)
        except KeyboardInterrupt:
            print("Keyboard Interrupt...")
            pass
        finally:
            # Make sure to stop notifications and disconnect properly
            await client.stop_notify(characteristic_uuid)

            n_timestamps = [datetime.strptime(ts, "%Y-%m-%d %H:%M:%S") for ts in timestamps]
            n_timestamps = [(dt - n_timestamps[0]).total_seconds() for dt in n_timestamps]

            print(n_timestamps)
            print(heart_rates)

            plt.figure(figsize=(10, 6))
            plt.plot(n_timestamps, heart_rates, marker='o', linestyle='-', color='blue')
            plt.title('Heart Rate vs. Time')
            plt.xlabel('Time')
            plt.ylabel('Heart Rate')
            plt.grid(True)
            plt.show()

if __name__ == "__main__":
    # Run the main function
    asyncio.run(main())