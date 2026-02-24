import depthai as dai
import time

print("Waiting for device to stabilize...")
time.sleep(2)

try:
    print("Getting available devices...")
    devices = dai.Device.getAllAvailableDevices()
    print(f"Found {len(devices)} devices")
    
    if devices:
        print("Attempting to connect...")
        device = dai.Device()
        print(f"SUCCESS! Connected to: {device.getMxId()}")
        print(f"Cameras: {device.getConnectedCameras()}")
        device.close()
    else:
        print("No devices found")
except Exception as e:
    print(f"Error: {e}")
    import traceback
    traceback.print_exc()