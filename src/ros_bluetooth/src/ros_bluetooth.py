#!/usr/bin/env python3

import asyncio
from bleak import BleakClient
from datetime import datetime
import matplotlib.pyplot as plt
import rospy
from std_msgs.msg import Int32

device_address = "A0:9E:1A:D3:66:6B"  # Replace with the address of your Bluetooth device
characteristic_uuid = "00002a37-0000-1000-8000-00805f9b34fb"  # Replace with the characteristic UUID for streaming data

heart_rates = list()
timestamps = list()

def notification_handler(sender, data):
    flags = data[0]

    if flags & 0x01:
        heart_rate = int.from_bytes(data[1:3], byteorder='little')
    else:
        heart_rate = data[1]

    now = datetime.now()
    timestamp = now.strftime("%Y-%m-%d %H:%M:%S")

    # print(f"{timestamp} - Received data from {sender}: Heart Rate Measurement: {heart_rate}")

    heart_rate_pub.publish(heart_rate)  # Publish heart rate to ROS

    heart_rates.append(heart_rate)
    timestamps.append(timestamp)

async def main(loop):
    async with BleakClient(device_address) as client:
        if not await client.is_connected():
            print("Failed to connect")
            return

        await client.start_notify(characteristic_uuid, notification_handler)

        print("Streaming data, press Ctrl+C to exit...")
        while not rospy.is_shutdown():
            await asyncio.sleep(1)

        # Stop notification before exiting
        await client.stop_notify(characteristic_uuid)

def shutdown_handler(loop):
    # Cancel all tasks running on the event loop
    for task in asyncio.all_tasks(loop):
        task.cancel()
    print("Shutdown initiated, stopping event loop...")

    # # Plotting code
    # if timestamps:
    #     n_timestamps = [datetime.strptime(ts, "%Y-%m-%d %H:%M:%S") for ts in timestamps]
    #     n_timestamps = [(dt - n_timestamps[0]).total_seconds() for dt in n_timestamps]

    #     plt.figure(figsize=(10, 6))
    #     plt.plot(n_timestamps, heart_rates, marker='o', linestyle='-', color='blue')
    #     plt.title('Heart Rate vs. Time')
    #     plt.xlabel('Time (seconds)')
    #     plt.ylabel('Heart Rate (bpm)')
    #     plt.grid(True)
    #     plt.show()
    # else:
    #     print("No data to plot.")

if __name__ == "__main__":
    # Initialize ROS node
    rospy.init_node('heart_rate_publisher', anonymous=True)
    heart_rate_pub = rospy.Publisher('heart_rate', Int32, queue_size=10)

    loop = asyncio.get_event_loop()
    # Register the shutdown handler
    rospy.on_shutdown(lambda: shutdown_handler(loop))
    try:
        loop.run_until_complete(main(loop))
    except rospy.ROSInterruptException:
        pass
    finally:
        loop.close()