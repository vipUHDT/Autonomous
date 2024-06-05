import FINAL_AUTONOMOUS_CLASS
import time
from dronekit import LocationGlobalRelative

test = FINAL_AUTONOMOUS_CLASS.CLASS()          #Runs the init function of the imported file


# test.dk_waypoint_lap()
# test.search_area_command()
# time.sleep( 5 )
test.deliver_payload_command()



print( "Mission Complete" )

