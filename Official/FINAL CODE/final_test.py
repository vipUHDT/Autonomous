import FINAL_AUTONOMOUS_CLASS

test = FINAL_AUTONOMOUS_CLASS.CLASS()          #Runs the init function of the imported file


test.connect_to_dronekit()

test.dk_waypoint_lap()

test.search_area_command()

test.deliver_payload_command()

print( "Mission Complete" )

