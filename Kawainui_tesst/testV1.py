import function
import time
UAS = function.CLASS()
start = time.time()
UAS.waypoint_lap()
print('waypoint lap finished')
UAS.search_area_waypoint()
print('Search the area finished. now doing waypoint lap again')
UAS.waypoint_lap()
print('waypoint lap finished')
UAS.average()
end = time.time()
print('complete')
diff = end-start
print(diff)