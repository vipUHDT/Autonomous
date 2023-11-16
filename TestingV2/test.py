import autonomous
import time
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command

fc = autonomous.CLASS()
print(fc.UAS_dk.mode)
fc.waypoint_lap()
# fc.UAS_dk.mode = VehicleMode("AUTO")

# fc.UAS_dk.mode = VehicleMode("GUIDED")
# fc.UAS_dk.mode = VehicleMode("AUTO")
#fc.UAS_dk.mode = VehicleMode("GUIDED")
# fc.UAS_dk.mode = VehicleMode("AUTO")


# fc.servo_command(5,fc.search_area_latitude[0],fc.search_area_longitude[0])
# fc.servo_command(6,fc.search_area_latitude[1],fc.search_area_longitude[1])
# fc.servo_command(7,fc.search_area_latitude[2],fc.search_area_longitude[2])

fc.search_area_waypoint()
#fc.UAS_dk.mode = VehicleMode("AUTO")
fc.waypoint_lap()
fc.export()



