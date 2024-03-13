import FINAL_AUTONOMOUS_CLASS

mission = FINAL_AUTONOMOUS_CLASS.CLASS()

mission.connect_to_dronekit()                       #Connect to DroneKit for waypoint lap and search area

mission.dk_waypoint_lap()                           #Conduct initial waypoint lap using dronekit

mission.search_area_command()                       #Conducts search area mission using dronekit

mission.deliver_payload_command()                   #Conducts payload delivery mission using dronekit and pymavlink

mission.waypoint_landing()                          #Flies to landing coordinate and hover. Descend manually to end mission