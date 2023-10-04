from pymavlink import mavutil

#UAS = mavutil.mavlink_connection('/dev/ttyACM0', baud=57600)
UAS = mavutil.mavlink_connection('COM7', baud=57600)
# Restart the ArduSub board !
#UAS.reboot_autopilot()
#waiting for heartbeat before sending commands
#UAS.wait_heartbeat()

# Define the waypoint parameters
waypoint_number = [1,2]  # You can increment this for each waypoint
latitude_deg = [0,0]  # Desired latitude in degrees
longitude_deg = [0,0]  # Desired longitude in degrees
altitude_m = 60  # Desired altitude in meters above ground level (AGL)
for x in range(size(waypoint_number)):
    # Create a waypoint command
    waypoint_command = mavutil.mavlink.MAV_CMD_NAV_WAYPOINT


    #parameter for waypoint
    INT_SEND_WAYPOINT_parameter = [UAS.target_system,  #target_system
        UAS.target_component, #target_component
        0, #frame try setting to 3 for more accuracy
        waypoint_command, #MAV_CMD_NAV_WAYPOINT (16 ) or try to change it to  waypoint_command
        0, #param1 hold: wait # time at position (seconds)
        10, #param2 Accept radius (m)
        0, #param3 pass radius (m)
        0, #param4 yaw (deg)
        latitude_deg[x], #current 
        longitude_deg[x],   #autocontinue
    altitude_m]     #param1
    
    #parameter for waypoint
    LONG_SEND_WAYPOINT_parameter = [UAS.target_system,  #target_system
        UAS.target_component, #target_component
        0, #frame try setting to 3 for more accuracy
        waypoint_command, #MAV_CMD_NAV_WAYPOINT (16) or try to change it to  waypoint_command
        1, #confirmation 
        0, #hold (s)
        10, #Accept radius (m)
        0, #pass radius (m)
        0, #yaw (deg)
        latitude_deg[x],  
        longitude_deg[x],  
    altitude_m]   

    # Send the waypoint command
    UAS.mav.command_int_send(
        *INT_SEND_WAYPOINT_parameter
    )
    
    # Send the waypoint command
    UAS.mav.command_long_send(
        *LONG_SEND_WAYPOINT_parameter
    )

    # Wait for confirmation (optional)
    print("Waypoint sent. Waiting for confirmation...")
    UAS.mav.wait_heartbeat()
    print("Waypoint confirmed.")