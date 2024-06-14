import serial
import threading
import time
from datetime import datetime

#output_name = 'output'
arduino_port = '/dev/tty.usbserial-AC013H1A'  # find port name on mac by going putting 'ls /dev/tty.*' in your terminal
baud_rate = 38400  # Baud rate

# Initialize serial connection
ser = serial.Serial(arduino_port, baud_rate, timeout=1)

def add_file_header(data_file, log_file):
    dt = datetime.now()
    
    # add data_file header
    data_file.write('Name: ' + output_name + ' data\n')
    data_file.write(dt.strftime("Date: %Y-%m-%d %H:%M:%S \n"))
    data_file.write('\n')
    data_file.write('Time (ms) \t X-Acc \t Y-Acc \t Z-Acc \n')

    # add log_file header
    log_file.write('Name: ' + output_name + ' log\n')
    log_file.write(dt.strftime("Date: %Y-%m-%d %H:%M:%S \n"))
    log_file.write('\n')
    log_file.write('Cmd \t Time (ms)\n')

def read_from_arduino():
    """Continuously read from Arduino"""
    log_filename = output_name + '_log.txt'
    data_filename = output_name + '_data.txt'

    with open(data_filename, 'a') as data_file, open(log_filename, 'a') as log_file: #a means append to existing file
        add_file_header(data_file, log_file)

        while not stop_thread.is_set(): #thread interrupt helps gracefully close things
            if ser.in_waiting > 0: #if anything new is written

                data = ser.readline().decode('utf-8').strip()
                print(data)
                if data[0].isdigit():
                    data_file.write(data + '\n')
                    data_file.flush()  # Ensure data is written to the file immediately
                else:
                    log_file.write(data + '\n')
                    log_file.flush()  # Ensure data is written to the file immediately

def write_to_arduino(data):
    """Write data to Arduino"""
    ser.write(data.encode())

def info_msg():
    info_msg = """Possible commands:
    a – Toggle LED on and off.
    r - reset
    x - read 1 sample off x, y, z acceleration
    t – temperature
    l - turn sensor on
    o - turn sensor off
    u - sensor frequency up
    d - sensor frequency down
    f - sensor frequency 4 (250 Hz)
    B - continuisously broadcast readings
    s - stop broadcasting readings
    ^C - exit program """
    print(info_msg)

if __name__ == "__main__":
    stop_thread = threading.Event() #essentially thread interrupt to help gracefully end program

    output_name = input("Run name: ")

    # Send user an info message
    info_msg()

    # Start the read thread
    read_thread = threading.Thread(target=read_from_arduino)
    read_thread.daemon = True
    read_thread.start()

    # Listen for writing from user
    try:
        while True:
            # Example: Prompt user to send data to Arduino
            data_to_send = input()
            write_to_arduino(data_to_send)
            time.sleep(1)

    except KeyboardInterrupt: #trigger with "control C" if you want to quit program
        print(f"\nProgram interrupted by the user")

    finally:
        stop_thread.set()
        read_thread.join()
        ser.close()