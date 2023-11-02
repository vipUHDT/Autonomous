from array import array

waypoint_lap_latitude = [
21.4002232, 21.4004455, 21.4007302, 21.4004929
]
waypoint_lap_longitude = [-157.7645463, -157.7646348, -157.7639616, -157.7637658]

#predefined search area value for Kawainui test
search_area_latitude = [
            21.4002344, 21.4003056, 21,4004118, 21.4004817, 
            21.4005753, 21.4007576, 21.4006702, 21.4006128, 
            21.4005541, 21.4004842, 21.4004530, 21.4005753
]

search_area_longitude = [
            -157.7644624, -157.7642258, -157.7640527, -157.7638744, 
            -157.7637121, -157.7638395, -157.7640112, -157.7641936,  
            -157.7643558, -157.7645986, -157.7642298, -157.7639160

]


def user_waypoint_input():
    """
    Allow the user to input a set of latitude and longitude coordinates for waypoints.

    Returns:
        None
    """
    # Ask for the number of coordinates and create a latitude and longitude array
    while 1:
        # Check for non-integer value
        try:
            number_of_coordinates = int(input("\nHow many coordinates?\n"))
            break
        except ValueError:
            print("Enter an integer")

    waypoint_lap_latitude  = array('i', [0] * number_of_coordinates)
    waypoint_lap_longitude = array('i', [0] * number_of_coordinates)

    # Ask for longitude and latitude coordinates and put them in their respective arrays
    for i in range(number_of_coordinates):
        while 1:
            # Check for non-integer values
            try:
                waypoint_lap_latitude [i] = int(input(f"Enter latitude {i + 1}:\n"))
                break
            except ValueError:
                print("Coordinate must be an integer")

        while 1:
            # Check for non-integer values
            try:
                waypoint_lap_longitude[i] = int(input(f"Enter longitude {i + 1}:\n"))
                break
            except ValueError:
                print("Coordinate must be an integer")

    # Print the coordinates in the array
    print("\nLatitudes entered:")
    for i in range(number_of_coordinates):
        if (i == number_of_coordinates-1):
            print(waypoint_lap_latitude [i])
        else:
            print(waypoint_lap_latitude [i], end=", ")
    print("\nLongitudes entered:")
    for i in range(number_of_coordinates):
        if (i == number_of_coordinates-1):
            print(waypoint_lap_longitude[i])
        else:
            print(waypoint_lap_longitude[i], end=", ") 

    while True:
        try:
            response = int(input("\nIS THE VALUE OF LATITUDE AND LONGITUDE CORRECT?\n1-YES or 2-NO\n"))
            if response in [1, 2]:
                if (response ==2):
                    user_waypoint_input()
                else:
                    break
            else:
                raise ValueError("\nInvalid response. Please enter 1-YES or 2-NO.")

        except ValueError as e:
            print(e)
        





user_waypoint_input()