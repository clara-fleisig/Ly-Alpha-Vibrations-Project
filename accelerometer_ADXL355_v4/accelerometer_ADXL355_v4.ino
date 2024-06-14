/*
  Reading 3 axis accelerometer from ADXL355
  By: Alvaro Nunes de Oliveira
  Date: 20 July 2022

  FINAL VERSION COMMAND LIST:
    The temperature on the Arduino Serial Plotter (Ctrl+Shit+L). But it is possible to plot pressure and humidity as well.
    Comand list:

    Executed only once
    a – Toggle LED on and off.
    r – reset
    x - read 1 sample off x, y, z acceleration
    t – temperature
    l - turn sensor on
    o - turn sensor off
    u - sensor frequency up
    d - sensor frequency down
    f - sensor frequency 4 (250 Hz)

    Kept in memory
    B - broadcast
    s - stop

    Reserved
    \*  - don't use this character

*/

#include <Wire.h>
#include "ADXL355_ANO.h"

adxl355 adxl355;

#define TCAADDR 0x70
#define FILTER_LOW_MAX 10

char   inChar;
char   cmd        = 's';
bool   time_aux   = false;
bool   led        = false;

uint8_t  filter_low = 4;
float    accX, accY, accZ;
float    temperature;

uint8_t  acc_byte[288];
uint8_t  *acc_byte_pointer = acc_byte;
int      acc_byte_length;
uint8_t  counter = 0;
uint8_t  fifo_entries;
uint8_t  fifo_samples;

adxl355_status status;

void setup(void)
{
  // Configure pins
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(PD2, OUTPUT);

  //Turn on the LED. LED should blink (1 second) to indicate a successful initialization.
  //After turning or reseting the Arduino, LED will blink a few times. After it, it will
  // be on for 1 second.
  digitalWrite(LED_BUILTIN, HIGH);

  Serial.begin(38400);

  //Default is Wire and is optional when you call .begin()
  Wire.begin();
  Wire.setClock(400000); //Communicate at 100kHz or 400kHz I2C

  // Rest MUX
  digitalWrite(PD2, LOW);
  delay(1);                   // Pulse duration RESET low: tW(L) > 6 ns
  digitalWrite(PD2, HIGH);
  delay(10);                  // Delay, for safety.

  if (false) { // to enable it, don't forget to remove the SDA jumper from the arduino to mux.
    Wire.beginTransmission(TCAADDR);
    Wire.write(0x02);
    Wire.endTransmission();
  }

  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  if (false) { // enable to debug initialization
    if (adxl355.is_connected()) Serial.print("ADXL okay.\n");
    else                        Serial.print("ADXL not connected!\n");

    Serial.print("Read register: \t");
    adxl355.config_fifo_samples(0x30);
    fifo_samples = adxl355.read_fifo_samples();
    Serial.print(fifo_samples, HEX); Serial.print("   ... done\n");

    adxl355.reset();
    Serial.print("reset ... done\n");

    Serial.print("Read register: \t");
    fifo_samples = adxl355.read_fifo_samples();
    Serial.print(fifo_samples, HEX); Serial.print("   ... done\n");
  }

  adxl355.reset();
  adxl355.config_filter(0, filter_low);
  adxl355.sensor_on();

  //Turn off the LED to indicate a successful sensor configuration.
  delay(1000);
  digitalWrite(LED_BUILTIN, LOW);
}


void loop(void)
{

  if (Serial.available()) {
    inChar = (char)Serial.read();
  }

  switch (inChar) {
    case 'a':
      led = !led;
      if (led) digitalWrite(LED_BUILTIN, HIGH);
      else     digitalWrite(LED_BUILTIN, LOW);
      break;

    case 'r': //flashes sensor
      digitalWrite(LED_BUILTIN, HIGH);
      adxl355.reset();
      digitalWrite(LED_BUILTIN, LOW);
      break;

    case 'x': //read and print accelerometer readings
      adxl355.read_acc_X(&accX);
      adxl355.read_acc_Y(&accY);
      adxl355.read_acc_Z(&accZ);
      Serial.print(millis()); Serial.print("\t");
      Serial.print(accX); Serial.print("\t");
      Serial.print(accY); Serial.print("\t");
      Serial.print(accZ); Serial.print("\n");
      break;

    case 't': //read and print timperature
      adxl355.read_temperature(&temperature);
      Serial.print("Temp. "); Serial.print(temperature); Serial.print("\t");
      Serial.print(millis());  Serial.print('\n');
      Serial.print("\n");
      break;

    case 'l':
      Serial.print("turn on\t");
      Serial.print(millis());  Serial.print('\n');
      adxl355.sensor_on();
      break;

    case 'o':
      Serial.print("turn off\t");
      Serial.print(millis());  Serial.print('\n');
      adxl355.sensor_off();
      break;

    case 'f':
      filter_low = 4;
      adxl355.sensor_off();
      adxl355.config_filter(0, filter_low);
      adxl355.sensor_on();
      Serial.print("sensor frequency 4 (250 Hz)\t");
      Serial.print(millis());  Serial.print('\n');
      break;

    case 'u':
      if (filter_low < FILTER_LOW_MAX) filter_low++;
      adxl355.sensor_off();
      adxl355.config_filter(0, filter_low);
      adxl355.sensor_on();
      Serial.print("sensor frequency up\t");
      Serial.print(millis());  Serial.print('\n');
      break;

    case 'd':
      if (filter_low > 0) filter_low--;
      adxl355.sensor_off();
      adxl355.config_filter(0, filter_low);
      adxl355.sensor_on();
      Serial.print("sensor frequency down\t");
      Serial.print(millis());  Serial.print('\n');
      break;

    case 'z':
      Serial.print(Serial.availableForWrite());
      Serial.print("\n");
      break;
    
    default:
      break;
  }


  if (inChar == 's') cmd = 's'; // stop
  if (inChar == 'B') cmd = 'B'; // boadcast

  switch (cmd) {

    case 'B':
      /*status = adxl355.read_acceleration_array_byte(acc_byte_pointer, &acc_byte_length, &fifo_entries);
      Serial.write((uint8_t)(acc_byte_length / 3));
      Serial.write(acc_byte, acc_byte_length);
      Serial.write(fifo_entries);
      counter++;*/
      adxl355.read_acceleration(&accX, &accY, &accZ);
      Serial.print(millis()); Serial.print("\t");
      Serial.print(accX); Serial.print("\t");
      Serial.print(accY); Serial.print("\t");
      Serial.print(accZ); Serial.print("\n");
      break;

    default:
      break;
  }

  inChar = '*'; // any character not used so far

}
