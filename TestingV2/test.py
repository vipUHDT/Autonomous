import autonomous
import time
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command

fc = autonomous.CLASS()
# while (fc.IS_ARMED() != True):
#     print("waiting to be armed")
#     print(fc.UAS_dk.armed)
#     time.sleep(1)
# while (fc.IS_GUIDED()  != True):
#     print("waiting to be in GUIDED mode")
#     print(fc.UAS_dk.mode)
#     time.sleep(1)
fc.mission_clear()
fc.spline_waypoint_lap()
fc.mission_clear()
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
fc.dk_waypoint_lap()
print("Finished with SPLINE")
for x in range(3):
    fc.deliver_payload_command(5,fc.search_area_latitude[x],fc.search_area_longitude[x])
fc.export()



