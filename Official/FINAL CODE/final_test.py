import FINAL_AUTONOMOUS_CLASS
import time
from dronekit import LocationGlobalRelative

test = FINAL_AUTONOMOUS_CLASS.CLASS()          #Runs the init function of the imported file


#go to waypoint and trigger servo test
# test.UAS_dk.simple_goto( LocationGlobalRelative( test.payload_delivery_latitude[0], test.payload_deliver_longitude[0], 26 ) )
# test.waypoint_reached( test.payload_delivery_latitude[0], test.payload_deliver_longitude[0], test.WAYPOINT_RADIUS )

# for i in range(4):
test.gpio_servo_command( 2, 120 )
#     time.sleep( 5 )
# print( "Mission Complete" )

#full mission test
# test.dk_waypoint_lap()
# test.search_area_command()
# test.deliver_payload_command()

print( "Mission Complete" )

