import FINAL_AUTONOMOUS_CLASS

test = FINAL_AUTONOMOUS_CLASS.CLASS()          #Runs the init function of the imported file

test.dk_waypoint_lap()

# drone.servo_command( 1, 1100 )                       #run waypoint lap using user input

# test.search_area_command()                     #run search area waypoints; coordinates are pre-defined

# drone.deliver_payload_command()

test.trigger_GoPro('test', False)
