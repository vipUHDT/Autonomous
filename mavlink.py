from pymavlink import mavutil

UAS = mavutil.mavlink_connection('/dev/ttyACM0', baud=57600)

# Restart the ArduSub board !
UAS.reboot_autopilot()
#waiting for heartbeat before sending commands
UAS.wait_heartbeat()

# Define the waypoint parameters
waypoint_number = [1,2]  # You can increment this for each waypoint
latitude_deg = [0,0]  # Desired latitude in degrees
longitude_deg = [0,0]  # Desired longitude in degrees
altitude_m = 60  # Desired altitude in meters above ground level (AGL)
for x in range(2):
    # Create a waypoint command
    waypoint_command = mavutil.mavlink.MAV_CMD_NAV_WAYPOINT
    params = [0, 0, 0, 0, latitude_deg[x], longitude_deg[x], altitude_m, 0, 0, 0, 0]

    # Send the waypoint command
    UAS.mav.command_long_send(
        UAS.target_system,
        UAS.target_component,
        waypoint_command,
        1,  # Confirmation code (set to 0 for no confirmation)
        waypoint_number,  # Waypoint number
        *params  # Waypoint parameters 
        #Note the '*' is used to unpack the elements in params list and pass the as seperate arguments to the command_long_send Method
    )

    # Wait for confirmation (optional)
    print("Waypoint sent. Waiting for confirmation...")
    UAS.mav.wait_heartbeat()
    print("Waypoint confirmed.")