import function_class_timer
import time

a = function_class_timer.CLASS()
start = time.time()
for x in range(30):
    a.trigger_camera()
    a.attitude()
    a.geotag()
end = time.time()
diff = end - start
print(diff)
a.average()