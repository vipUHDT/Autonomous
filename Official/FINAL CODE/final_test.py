import FINAL_AUTONOMOUS_CLASS
import time
from dronekit import LocationGlobalRelative

test = FINAL_AUTONOMOUS_CLASS.CLASS()          #Runs the init function of the imported file

test.connect_to_dronekit()
test.UAS_dk.simple_goto(LocationGlobalRelative( test.payload_delivery_latitude[0], test.payload_deliver_longitude[0], 26 ) )

test.servo_command( 1, 700 )
time.sleep( 3 )
test.servo_command( 2, 700 )
time.sleep( 3 )
test.servo_command( 3, 700 )
time.sleep( 3 )
test.servo_command( 4, 700 )
# test.connect_to_dronekit()

# test.dk_waypoint_lap()
# # test.search_area_command()

# test.deliver_payload_command()
# test.export()

print( "Mission Complete" )

