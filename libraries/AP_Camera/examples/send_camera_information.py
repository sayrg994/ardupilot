import threading
import time
from pymavlink import mavutil

ONBOARD_CONTROLLER_SYSID = 245
ONBOARD_CONTROLLER_COMPID = 0

# Function to connect to MAVLink
def connect_to_mavlink(ip, port):
    connection = mavutil.mavlink_connection(f'udp:{ip}:{port}', source_system=ONBOARD_CONTROLLER_SYSID)
    connection.wait_heartbeat()
    print("Heartbeat received from system (system %u component %u)" % (connection.target_system, connection.target_component))
    # Send a heartbeat back to register the device
    connection.mav.heartbeat_send(
        type=mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
        autopilot=mavutil.mavlink.MAV_AUTOPILOT_INVALID,
        base_mode=0,
        custom_mode=0,
        system_status=mavutil.mavlink.MAV_STATE_UNINIT,
        mavlink_version=3
    )
    return connection

ip = "127.0.0.1"
port = 14570

# Connect to MAVLink
connection = connect_to_mavlink(ip, port)

# Function to send the camera information message
def send_camera_information(connection):
    time_boot_ms = int(time.time() * 1000) & 0xFFFFFFFF  # Current time in milliseconds since boot

    # Dummy data for camera information
    vendor_name = b"CameraVendor123" + b'\0' * (32 - len("CameraVendor123"))
    model_name = b"CameraModel123" + b'\0' * (32 - len("CameraModel123"))
    firmware_version = (1 << 24) | (0 << 16) | (0 << 8) | 1  # Example: version 1.0.0.1
    focal_length = float('nan')  # Not known
    sensor_size_h = float('nan')  # Not known
    sensor_size_v = float('nan')  # Not known
    resolution_h = 640  # Example resolution
    resolution_v = 480  # Example resolution
    lens_id = 0  # Not known
    flags = 4095  # All capabilities
    cam_definition_version = 0  # Not known
    cam_definition_uri = b""  # Not known
    gimbal_device_id = 0  # Not known

    # Send camera information
    connection.mav.camera_information_send(
        time_boot_ms,
        vendor_name,
        model_name,
        firmware_version,
        focal_length,
        sensor_size_h,
        sensor_size_v,
        resolution_h,
        resolution_v,
        lens_id,
        flags,
        cam_definition_version,
        cam_definition_uri,
        gimbal_device_id
    )
    print("Camera information message sent")

# Define a function to handle the MAV_CMD_CAMERA_TRACK_POINT command
def handle_camera_track_point(msg):
    print("Received MAV_CMD_CAMERA_TRACK_POINT command.")
    param1 = msg.param1  # Example parameter, adjust as needed
    param2 = msg.param2  # Example parameter, adjust as needed
    # Parse other necessary parameters from the msg
    print(f"Tracking point parameters: param1={param1}, param2={param2}")

def send_heartbeat(connection):
    while True:
        connection.mav.heartbeat_send(
            type=mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
            autopilot=mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            base_mode=0,
            custom_mode=0,
            system_status=mavutil.mavlink.MAV_STATE_UNINIT,
            mavlink_version=3
        )
        time.sleep(1)

# Main function
def main():
    # Send camera information message
    send_camera_information(connection)
    print('sent CAMERA INFO')
    
    # Start listening for MAVLink messages
    while True:
        msg = connection.recv_match(type='COMMAND_LONG', blocking=True)
        if msg:
            # Check if the message is a COMMAND_LONG
            if msg.get_type() == 'COMMAND_LONG' and msg.command == mavutil.mavlink.MAV_CMD_CAMERA_TRACK_POINT:
                if msg.target_system == ONBOARD_CONTROLLER_SYSID:
                    handle_camera_track_point(msg)
                else:
                    print("Received but not for us")

if __name__ == "__main__":
    main()