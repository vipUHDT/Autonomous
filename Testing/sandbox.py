import math
def toRadian(degree):
    pi = math.pi
    return degree * (pi / 180)

def haversine(lat1, lon1, lat2, lon2):
    #curr_location = UAS.location.global_relative_frame
    lat1 = toRadian(lat1)
    lon1 = toRadian(lon1)
    lat2 = toRadian(lat2)
    lon2 = toRadian(lon2)
    #lat2 = toRadian(curr_location.latitude)
    #lon2 = toRadian(curr_location.longitude)

    diff_lat = lat2 - lat1
    diff_lon = lon2 - lon1

    return 5280 * 3963.0 * math.acos( (math.sin(lat1)*math.sin(lat2)) + (math.cos(lat1) * math.cos(lat2)) * math.cos(lon2 - lon1) )

x = haversine(21.4002476,-157.7643880,21.4001414,-157.7646495)
print(x)       