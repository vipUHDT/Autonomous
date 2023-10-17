from array import array

def user_waypoint_input():
    # Ask for the number of coordinates and create a latitude and longitude array
    while 1:
        # Check for non-integer value
        try:
            number_of_coordinates = int(input("How many coordinates?\n"))
            break
        except ValueError:
            print("Enter an integer")

    latitude_array = array('i', [0] * number_of_coordinates)
    longitude_array = array('i', [0] * number_of_coordinates)

    # Ask for longitude and latitude coordinates and put them in their respective arrays
    for i in range(number_of_coordinates):
        while 1:
            # Check for non-integer values
            try:
                latitude_array[i] = int(input(f"Enter latitude {i + 1}:\n"))
                break
            except ValueError:
                print("Coordinate must be an integer")

        while 1:
            # Check for non-integer values
            try:
                longitude_array[i] = int(input(f"Enter longitude {i + 1}:\n"))
                break
            except ValueError:
                print("Coordinate must be an integer")

    # Print the coordinates in the array
    print("\nLatitudes entered:")
    for i in range(number_of_coordinates):
        if (i == number_of_coordinates-1):
            print(latitude_array[i])
        else:
            print(latitude_array[i], end=", ")
        

    print("\nLongitudes entered:")
    for i in range(number_of_coordinates):
        if (i == number_of_coordinates-1):
            print(longitude_array[i])
        else:
            print(longitude_array[i], end=", ") 

user_waypoint_input()