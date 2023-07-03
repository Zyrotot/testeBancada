import serial
from datetime import datetime

def save_output(output):
    timestamp = datetime.now().strftime("%Y%m%d%H%M")
    with open(f"output_{timestamp}.txt", "w") as file:
        file.write(output)

# Establish serial communication with the Arduino
ser = serial.Serial('COM3', 9600)  # Replace 'COM3' with the appropriate serial port

# Send command '7' to the Arduino to run all tests
ser.write(b'7\n')

# Initialize variables
output = ""

while True:
    # Read data from the serial port
    data = ser.readline().decode().strip()
    
    # Append the received data to the output string
    output += data + "\n"
    
    # Save the output to a file with the current timestamp
    save_output(output)
    
    # Check if the line "All tests completed." is received
    if data == "All tests completed.":
        break

# Close the serial connection
ser.close()
