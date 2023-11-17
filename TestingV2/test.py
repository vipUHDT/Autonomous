import autonomous
import time
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command

fc = autonomous.CLASS()

fc.dk_waypoint_lap()


fc.search_area_waypoint()
fc.dk_waypoint_lap()
print("Finished with SPLINE")
for x in range(3):
    fc.deliver_payload_command(5,fc.search_area_latitude[x],fc.search_area_longitude[x])
fc.export()



