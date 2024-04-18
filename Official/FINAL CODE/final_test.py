import FINAL_AUTONOMOUS_CLASS
import time
from dronekit import LocationGlobalRelative

test = FINAL_AUTONOMOUS_CLASS.CLASS()          #Runs the init function of the imported file

test.gpio_servo_command( 0, 120)

print( "Mission Complete" )

