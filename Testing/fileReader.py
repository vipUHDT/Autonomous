import pandas as pd

def read_csv_coordinates(file_path):
    try:
        # Read the csv file
        df = pd.read_csv(file_path)
        
        # Extract latitude and longitude columns
        latitude_values = df['Latitude'].values
        longitude_values = df['Longitude'].values
        servo_numbers = df['Servo'].values

        return latitude_values, longitude_values, servo_numbers
    
    except Exception as e:
        print("An error occurred:", e)
        return None, None


file_path = "Test_Data_CSV1.csv"  #file path for data
latitude_values, longitude_values, servo_numbers = read_csv_coordinates(file_path)

if latitude_values is not None and longitude_values is not None and servo_numbers is not None:
    print("Latitude values:", latitude_values)
    print("Longitude values:", longitude_values)
    print("Servo numbers:", servo_numbers)
else:
    print("Failed to read coordinates from the CSV file.")
