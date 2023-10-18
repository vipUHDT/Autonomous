from array import array

def user_waypoint_input():
    # Ask for the number of coordinates and create a latitude and longitude array
    while 1:
        # Check for non-integer value
        try:
            number_of_coordinates = int(input("万歳 How many coordinates?\n"))
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
            print(waypoint_lap_longitude[i], "\n")
        else:
            print(waypoint_lap_longitude[i], end=", ") 

user_waypoint_input()
while True:
    try:
        response = int(input("IS THE VALUE OF LATITUDE AND LONGITUDE CORRECT?\n1-YES or 2-NO\n"))
        if response in [1, 2]:
            if (response ==2):
                user_waypoint_input()
            else:
                break
        else:
            raise ValueError("\nInvalid response. Please enter 1-YES or 2-NO.")

    except ValueError as e:
        print(e)



