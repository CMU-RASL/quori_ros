## Heart Rate Capturing via Bluetooth

### Usage
- ```python find_bluetooth.py``` displays all nearby bluetooth devices. Find the one that says "Device found: Polar Sense" and enter its address as a string for the device_address in look_up_char_uuid.py
- ```python look_up_char_uuid.py``` displays the chararacteristic id for all services on that device. We are interested in the heart rate service only. Find the characteristic id under the service titled "Heart Rate." Enter both the address and the char id as device_address and characteristic_uuid in connect_bluetooth.py
- ```python connect_bluetooth.py``` reads the heart rate data and prints in the console. After ctrl+c, the program prints out the timestamps and heart rates and dispalys a graph. The timestamps and heart rates can be copied for later analysis.

### ROS
- ```rosrun ros_bluetooth ros_bluetooth.py``` has the same functionalities as connect_bluetooth.py with ROS integrated.

### Dependencies
- ROS (refer to the official ROS website)
- Bleak (pip install bleak)
- Matplotlib (pip install matplotlib)