# Ly-Alpha-Vibrations-Project
## Overview
This project is associated with my work as an Azuma Fellow in the ALPHA experiment at CERN, working on the Lyman-Alpha laser team. The goal of this project was to track down a suspected vibrational resonances of one of the mirrors which was causing instabilites in the experimental setup. This involved measuring vibrations using an ADXL355 accelerometer which was connected to an I2C bus and an Arduino Nano.

## Arduino Program
The code used for the Arduino is a modified version of the C++ code written by Álvaro Nunes De Oliveira. Modifications Álvaros original code include the addition of timestamps and printed messages to the wire transferred data, as well as a modifiction of the FIFO queue reading method so as not to require tweaking of the wire.h library. 

This code can be uploaded to the Arduino Nano by openning the 'accelerometer_ADXL355_v4' directory in the Arduino IDE.

## Data
For data curation a multi-threaded python script (Data/curate_data.py) was used to listen and write to the Arduino port. Note that you may need to modify the arduino port name at the top of the python script depending on the port your arduino is connected to. This script allows the user to send various commands to the arduino, records accelerometer data in a file with a "_data.txt" suffix, and records settings modifiction made to the arduino program in a file with a "_log.txt" suffix. 

## Analysis
Coming soon...
