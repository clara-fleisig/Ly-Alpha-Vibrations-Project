#include "ADXL355_ANO.h"
#include <Wire.h>

// Accelerometer device address
#define ADXL355_ADDR 0x1D

// Registers address
#define REG_DEVID_AD     0x00
#define REG_DEVID_MST    0x01
#define REG_PARTID       0x02
#define REG_REVID        0x03
#define REG_STATUS       0x04
#define REG_FIFO_ENTRIES 0x05
#define REG_TEMP2        0x06
#define REG_TEMP1        0x07
#define REG_XDATA3       0x08
#define REG_XDATA2       0x09
#define REG_XDATA1       0x0A
#define REG_YDATA3       0x0B
#define REG_YDATA2       0x0C
#define REG_YDATA1       0x0D
#define REG_ZDATA3       0x0E
#define REG_ZDATA2       0x0F
#define REG_ZDATA1       0x10
#define REG_FIFO_DATA    0x11
#define REG_OFFSET_X_H   0x1E
#define REG_OFFSET_X_L   0x1F
#define REG_OFFSET_Y_H   0x20
#define REG_OFFSET_Y_L   0x21
#define REG_OFFSET_Z_H   0x22
#define REG_OFFSET_Z_L   0x23
#define REG_ACT_EN       0x24
#define REG_ACT_THRESH_H 0x25
#define REG_ACT_THRESH_L 0x26
#define REG_ACT_COUNT    0x27
#define REG_FILTER       0x28
#define REG_FIFO_SAMPLES 0x29
#define REG_INT_MAP      0x2A
#define REG_SYNC         0x2B
#define REG_RANGE        0x2C
#define REG_POWER_CTL    0x2D
#define REG_SELF_TEST    0x2E
#define REG_RESET        0x2F
#define REG_SHADOW       0x50

// Commands
#define CMD_RESET        0X52

// Masks
#define MSK_FIFO_ENTRIES 0x7F
#define MSK_FIFO_SAMPLES 0x7F

uint8_t  shadow_reg_constant[5] = {0xD5, 0x05, 0x02, 0x84, 0x20}; // DO NOT CHANGE THIS VALUES!
uint8_t  shadow_reg;

/********************************************************************* *******************************************************************
   \brief Perform initial configuration. Has to be called once.
*/
void adxl355::begin(void) {
  Wire.begin();
}

/*********************************************************************
   \brief Check whether adxl355 device is connected

   \return bool : status of adxl355
         - true : Device is present
         - false : Device is not acknowledging I2C address
*/
bool adxl355::is_connected(void) {
  Wire.beginTransmission(ADXL355_ADDR);
  return (Wire.endTransmission() == 0);
}


/********************************************************************* *******************************************************************
   \brief Reset the adxl355 device

   \return adxl355_status : status of adxl355
         - adxl355_status_ok : I2C transfer completed successfully
         - adxl355_status_i2c_transfer_error : Problem with i2c transfer
         - adxl355_status_no_i2c_acknowledge : I2C did not acknowledge
*/
enum adxl355_status adxl355::reset(void) {
  enum adxl355_status status;
  bool check_0x50 = false;
  bool check_0x51 = false;
  bool check_0x52 = false;
  bool check_0x53 = false;
  bool check_0x54 = false;

  do {
    write_user_register(REG_RESET, CMD_RESET);
    delay(10);
    // reading shadow registers, see data sheet observation about reset function race condition
    read_user_register(REG_SHADOW + 0, &shadow_reg); if (shadow_reg == shadow_reg_constant[0]) check_0x50 = true; 
    read_user_register(REG_SHADOW + 1, &shadow_reg); if (shadow_reg == shadow_reg_constant[1]) check_0x51 = true;
    read_user_register(REG_SHADOW + 2, &shadow_reg); if (shadow_reg == shadow_reg_constant[2]) check_0x52 = true;
    read_user_register(REG_SHADOW + 3, &shadow_reg); if (shadow_reg == shadow_reg_constant[3]) check_0x53 = true;
    read_user_register(REG_SHADOW + 4, &shadow_reg); if (shadow_reg == shadow_reg_constant[4]) check_0x54 = true;
  } while (!(check_0x50 && check_0x51 && check_0x52 && check_0x53 && check_0x54));

  return adxl355_status_ok;
}


/*********************************************************************
   \brief Read the temperature.

   \param[out] float* : sensor temperature (oC)

   \return adxl355_status : status of adxl355
         - adxl355_status_ok : I2C transfer completed successfully
         - adxl355_status_i2c_transfer_error : Problem with i2c transfer
         - adxl355_status_no_i2c_acknowledge : I2C did not acknowledge
         - adxl355_status_crc_error : CRC check error
*/
enum adxl355_status adxl355::read_temperature(float *t) {
  enum adxl355_status status;
  uint8_t i2c_status;
  uint8_t buffer[2];
  buffer[0] = 0;
  buffer[1] = 0;

  status = read_user_register(REG_TEMP2, &buffer[0]);
  if (status != adxl355_status_ok)
    return status;
  status = read_user_register(REG_TEMP1, &buffer[1]);
  if (status != adxl355_status_ok)
    return status;

  *t = (1885 - (((uint32_t)buffer[0] << 8) | (uint32_t)buffer[1])) / 9.05 + 25;
  //  c=(t[0]<<8)+t[1];
  //  c=((1852 - c)/9.05)+25.7;

  return adxl355_status_ok;
}


/****************************************************************
    \brief Reads the fifo entries.

    \return int numbers of entries (0 to 127)
*/
int adxl355::fifo_entries() {
  enum adxl355_status status;
  uint8_t i2c_status;
  uint8_t buffer;

  status = read_user_register(REG_FIFO_ENTRIES, &buffer);
  if (status != adxl355_status_ok)
    return 0;

  return buffer & MSK_FIFO_ENTRIES;
}


/****************************************************************
    \brief Reads the fifo samples.

    \return int numbers of samples
*/
int adxl355::read_fifo_samples() {
  enum adxl355_status status;
  uint8_t i2c_status;
  uint8_t buffer;

  status = read_user_register(REG_FIFO_SAMPLES, &buffer);
  if (status != adxl355_status_ok)
    return 0;

  return buffer & MSK_FIFO_SAMPLES;
}


/****************************************************************
    \brief Reads the acceleration in fifo register.

    \param[out] float* : acceleration x
    \param[out] float* : acceleration y
    \param[out] float* : acceleration z

    \return adxl355_status : status of adxl355
          - adxl355_status_ok : I2C transfer completed successfully
          - adxl355_status_i2c_transfer_error : Problem with i2c transfer
          - adxl355_status_no_i2c_acknowledge : I2C did not acknowledge
          - adxl355_status_crc_error : CRC check error
*/
enum adxl355_status adxl355::read_acceleration(float *x, float *y, float *z) {
  enum adxl355_status status;
  uint8_t i2c_status;
  uint8_t buffer[288];

  // Send the Read Register Command
  Wire.beginTransmission((uint8_t)ADXL355_ADDR);
  Wire.write(REG_FIFO_DATA);
  i2c_status = Wire.endTransmission();

  Wire.requestFrom((uint8_t)ADXL355_ADDR, 288U);
  for (int i = 0; i < 288; i += 3) {
    buffer[i  ] = Wire.read();
    buffer[i + 1] = Wire.read();
    buffer[i + 2] = Wire.read();
    if (buffer[i + 2] == 2) break;
  }

  *x = ((uint32_t)buffer[0] << 12) | ((uint32_t)buffer[1] << 4) | ((uint32_t)buffer[2] >> 4);
  *y = ((uint32_t)buffer[3] << 12) | ((uint32_t)buffer[4] << 4) | ((uint32_t)buffer[5] >> 4);
  *z = ((uint32_t)buffer[6] << 12) | ((uint32_t)buffer[7] << 4) | ((uint32_t)buffer[8] >> 4);

  if ( ((buffer[2] & 0x03) == 1) & ((buffer[5] & 0x03) == 0) & ((buffer[8] & 0x03) == 0) ) {
  }
  else {
    return adxl355_status_fifo_error;
  }

  return adxl355_status_ok;
}


/****************************************************************
    \brief Reads the acceleration in fifo register.

    \param[in] float* : pointer to acc vector

    \return adxl355_status : status of adxl355
          - adxl355_status_ok : I2C transfer completed successfully
          - adxl355_status_i2c_transfer_error : Problem with i2c transfer
          - adxl355_status_no_i2c_acknowledge : I2C did not acknowledge
          - adxl355_status_crc_error : CRC check error
*/
enum adxl355_status adxl355::read_acceleration_array(float *p) {
  enum adxl355_status status;
  uint8_t i2c_status;
  uint8_t byte0, byte1, byte2;

  // Send the Read Register Command
  Wire.beginTransmission((uint8_t)ADXL355_ADDR);
  Wire.write(REG_FIFO_DATA);
  i2c_status = Wire.endTransmission();

  Wire.requestFrom((uint8_t)ADXL355_ADDR, 288U); // = 96 x 3
  for (int i = 0; i < 96; i++) {
    byte0 = Wire.read();
    byte1 = Wire.read();
    byte2 = Wire.read();
    if ((byte2 == 2)) break;
    *(p + i) = ((uint32_t)byte0 << 12) | ((uint32_t)byte1 << 4) | ((uint32_t)byte2 >> 4);
  }

  //  if ( ((buffer[2]&0x03)==1) & ((buffer[5]&0x03)==0) & ((buffer[8]&0x03)==0) ) {
  //    }
  //  else {
  //    return adxl355_status_fifo_error;
  //  }

  return adxl355_status_ok;
}

/****************************************************************
    \brief Reads the acceleration in fifo register.

    \param[out] uint8_t* : pointer to acc byte vector
    \param[out] int*     : pointer length

    \return adxl355_status : status of adxl355
          - adxl355_status_ok : I2C transfer completed successfully
          - adxl355_status_i2c_transfer_error : Problem with i2c transfer
          - adxl355_status_no_i2c_acknowledge : I2C did not acknowledge
          - adxl355_status_crc_error : CRC check error
*/
enum adxl355_status adxl355::read_acceleration_array_byte(uint8_t *p, int *number_bytes, uint8_t *num_entries) {
  enum adxl355_status status;
  uint8_t i2c_status;
  uint8_t byte[288];
  uint8_t aux;
  int i;
  int j;
  int bytes_to_read;

  aux = fifo_entries();
  *num_entries = aux;
  bytes_to_read = (int)(aux * 3);
  *number_bytes = bytes_to_read;

  // Send the Read Register Command
  //Wire.beginTransmission((uint8_t)ADXL355_ADDR);
  //Wire.write(REG_FIFO_DATA);
  //Wire.endTransmission();

  int num_d = bytes_to_read / 27;
  int num_r = bytes_to_read % 27;

  for (j = 0; j < (bytes_to_read - num_r); j = j + 27) {
    Wire.beginTransmission((uint8_t)ADXL355_ADDR);
    Wire.write(REG_FIFO_DATA);
    Wire.endTransmission();
    Wire.requestFrom((uint8_t)ADXL355_ADDR, (uint8_t)27);
    for (i = 0; i < 27; i++)  {
      *(p + j + i) = Wire.read();
      //byte[j + i] = Wire.read();
    }
  }
  if (num_r != 0) {
    Wire.beginTransmission((uint8_t)ADXL355_ADDR);
    Wire.write(REG_FIFO_DATA);
    Wire.endTransmission();
    Wire.requestFrom((uint8_t)ADXL355_ADDR, (uint8_t)num_r);
    for (i = 0; i < num_r; i++) {
      *(p + bytes_to_read - num_r + i) = Wire.read();
      //byte[bytes_to_read - num_r + i] = Wire.read();
    }
  }

  return adxl355_status_ok;
}


/****************************************************************
    \brief Reads the acceleration in fifo register.

    \param[out] uint8_t* : pointer to acc byte vector
    \param[out] int*     : pointer length

    \return adxl355_status : status of adxl355
          - adxl355_status_ok : I2C transfer completed successfully
          - adxl355_status_i2c_transfer_error : Problem with i2c transfer
          - adxl355_status_no_i2c_acknowledge : I2C did not acknowledge
          - adxl355_status_crc_error : CRC check error
*/
enum adxl355_status adxl355::read_acceleration_array_byte_test(uint8_t *p, int *number_bytes, uint8_t *num_entries, int count) {
  enum adxl355_status status;
  uint8_t i2c_status;
  uint8_t byte[288];
  uint8_t aux;
  int i;
  int j;
  int bytes_to_read;

  aux = fifo_entries();
  *num_entries = aux;
  aux = count;
  bytes_to_read = (int)(aux * 3);
  *number_bytes = bytes_to_read;

  int num_d = bytes_to_read / 27;
  int num_r = bytes_to_read % 27;

  for (j = 0; j < (bytes_to_read - num_r); j = j + 27) {
    Wire.beginTransmission((uint8_t)ADXL355_ADDR);
    Wire.write(REG_FIFO_DATA);
    Wire.endTransmission();
    Wire.requestFrom((uint8_t)ADXL355_ADDR, (uint8_t)27);
    for (i = 0; i < 27; i++)  {
      *(p + j + i) = Wire.read();
      //byte[j + i] = Wire.read();
      //Serial.print(j + i); Serial.print(" ");
    }
  }
  if (num_r != 0) {
    Wire.beginTransmission((uint8_t)ADXL355_ADDR);
    Wire.write(REG_FIFO_DATA);
    Wire.endTransmission();
    Wire.requestFrom((uint8_t)ADXL355_ADDR, (uint8_t)num_r);
    for (i = 0; i < num_r; i++) {
      *(p + bytes_to_read - num_r + i) = Wire.read();
      //byte[bytes_to_read - num_r + i] = Wire.read();
      //Serial.print(bytes_to_read - num_r + i); Serial.print(" ");
    }
  }
  //Serial.print("\n");

  /*
    for (i = 0; i < bytes_to_read; i = i + 9) {
      Serial.print(i / 9 + 1);                                                                                   Serial.print("\t");
      Serial.print(((uint32_t)byte[i + 0] << 12) | ((uint32_t)byte[i + 1] << 4) | ((uint32_t)byte[i + 2] >> 4)); Serial.print("\t");
      Serial.print(((uint32_t)byte[i + 3] << 12) | ((uint32_t)byte[i + 4] << 4) | ((uint32_t)byte[i + 5] >> 4)); Serial.print("\t");
      Serial.print(((uint32_t)byte[i + 6] << 12) | ((uint32_t)byte[i + 7] << 4) | ((uint32_t)byte[i + 8] >> 4)); Serial.print("\t");
      Serial.print(bytes_to_read);                                                                               Serial.print("\t");
      Serial.print(aux);                                                                                         Serial.print("\t");
      Serial.print(count / 3);                                                                                   Serial.print("\n");
    }
    Serial.print("\n");*/
  return adxl355_status_ok;
}


/****************************************************************
   \brief Reads the acceleration X.

   \param[out] float* : acceleration x

   \return adxl355_status : status of adxl355
         - adxl355_status_ok : I2C transfer completed successfully
         - adxl355_status_i2c_transfer_error : Problem with i2c transfer
         - adxl355_status_no_i2c_acknowledge : I2C did not acknowledge
         - adxl355_status_crc_error : CRC check error
*/
enum adxl355_status adxl355::read_acc_X(float *x) {
  enum adxl355_status status;
  uint8_t i2c_status;
  uint8_t buffer[3];

  read_user_register(REG_XDATA3, &buffer[0]);
  read_user_register(REG_XDATA2, &buffer[1]);
  read_user_register(REG_XDATA1, &buffer[2]);

  *x = (((uint32_t)buffer[0] << 16) | ((uint32_t)buffer[1] << 8) | ((uint32_t)buffer[2] & 0xF0)) >> 4;

  return adxl355_status_ok;
}


/****************************************************************
   \brief Reads the acceleration Y.

   \param[out] float* : acceleration y

   \return adxl355_status : status of adxl355
         - adxl355_status_ok : I2C transfer completed successfully
         - adxl355_status_i2c_transfer_error : Problem with i2c transfer
         - adxl355_status_no_i2c_acknowledge : I2C did not acknowledge
         - adxl355_status_crc_error : CRC check error
*/
enum adxl355_status adxl355::read_acc_Y(float *y) {
  enum adxl355_status status;
  uint8_t i2c_status;
  uint8_t buffer[3];

  read_user_register(REG_YDATA3, &buffer[0]);
  read_user_register(REG_YDATA2, &buffer[1]);
  read_user_register(REG_YDATA1, &buffer[2]);

  *y = (((uint32_t)buffer[0] << 16) | ((uint32_t)buffer[1] << 8) | ((uint32_t)buffer[2] & 0xF0)) >> 4;

  return adxl355_status_ok;
}


/****************************************************************
   \brief Reads the acceleration Z.

   \param[out] float* : acceleration z

   \return adxl355_status : status of adxl355
         - adxl355_status_ok : I2C transfer completed successfully
         - adxl355_status_i2c_transfer_error : Problem with i2c transfer
         - adxl355_status_no_i2c_acknowledge : I2C did not acknowledge
         - adxl355_status_crc_error : CRC check error
*/
enum adxl355_status adxl355::read_acc_Z(float *z) {
  enum adxl355_status status;
  uint8_t i2c_status;
  uint8_t buffer[3];

  read_user_register(REG_ZDATA3, &buffer[0]);
  read_user_register(REG_ZDATA2, &buffer[1]);
  read_user_register(REG_ZDATA1, &buffer[2]);

  *z = (((uint32_t)buffer[0] << 16) | ((uint32_t)buffer[1] << 8) | ((uint32_t)buffer[2] & 0xF0)) >> 4;

  return adxl355_status_ok;
}


/****************************************************************
    \brief Config filter (ODR).

    \param[in] uint8_t* : high pass
    \param[in] uint8_t* : low pass

    \return adxl355_status : status of adxl355
          - adxl355_status_ok : I2C transfer completed successfully
          - adxl355_status_i2c_transfer_error : Problem with i2c transfer
          - adxl355_status_no_i2c_acknowledge : I2C did not acknowledge
          - adxl355_status_crc_error : CRC check error
*/
enum adxl355_status adxl355::config_filter(uint8_t hpf, uint8_t lpf) {
  enum adxl355_status status;
  status = write_user_register(REG_FILTER, ((hpf & 0x07) << 4) | (lpf & 0x0F) );
  return adxl355_status_ok;
}


/****************************************************************
    \brief Config FIFO samples.

    \param[in] uint8_t* : fifo sample size

    \return adxl355_status : status of adxl355
          - adxl355_status_ok : I2C transfer completed successfully
          - adxl355_status_i2c_transfer_error : Problem with i2c transfer
          - adxl355_status_no_i2c_acknowledge : I2C did not acknowledge
          - adxl355_status_crc_error : CRC check error
*/
enum adxl355_status adxl355::config_fifo_samples(uint8_t samples) {
  enum adxl355_status status;
  status = write_user_register(REG_FIFO_SAMPLES, samples);
  return adxl355_status_ok;
}


/****************************************************************
   \brief Turn on acc sensor.

   \return adxl355_status : status of adxl355
         - adxl355_status_ok : I2C transfer completed successfully
         - adxl355_status_i2c_transfer_error : Problem with i2c transfer
         - adxl355_status_no_i2c_acknowledge : I2C did not acknowledge
         - adxl355_status_crc_error : CRC check error
*/
enum adxl355_status adxl355::sensor_on() {
  enum adxl355_status status;
  status = write_user_register(REG_POWER_CTL, 2); // senor measurement mode, temberature sesnor disable
  status = write_user_register(REG_POWER_CTL, 0); // senor measurement mode, temberature sensor enable
  return adxl355_status_ok;
}

/****************************************************************
   \brief Turn off acc sensor (stand by)

   \return adxl355_status : status of adxl355
         - adxl355_status_ok : I2C transfer completed successfully
         - adxl355_status_i2c_transfer_error : Problem with i2c transfer
         - adxl355_status_no_i2c_acknowledge : I2C did not acknowledge
         - adxl355_status_crc_error : CRC check error
*/
enum adxl355_status adxl355::sensor_off() {
  enum adxl355_status status;
  status = write_user_register(REG_POWER_CTL, 1); // senor standby
  return adxl355_status_ok;
}


/*********************************************************************
   \brief Read register value

   \param[in]  uint8_t  : register address.
   \param[in]  uint8_t  : number of bytes to be read.
   \param[out] uint8_t* : register value.

   \return adxl355_status : status of adxl355
         - adxl355_status_ok : I2C transfer completed successfully
         - adxl355_status_i2c_transfer_error : Problem with i2c transfer
         - adxl355_status_no_i2c_acknowledge : I2C did not acknowledge
*/
enum adxl355_status adxl355::read_user_register(uint8_t reg, uint8_t *value) {
  enum adxl355_status status;
  uint8_t i2c_status;
  uint8_t buffer;

  // Send the Read Register Command
  Wire.beginTransmission((uint8_t)ADXL355_ADDR);
  Wire.write(reg);
  i2c_status = Wire.endTransmission();

  Wire.requestFrom((uint8_t)ADXL355_ADDR, 1U);
  buffer = Wire.read();


  if (status != adxl355_status_ok)
    return status;

  if (i2c_status == i2c_status_err_overflow)
    return adxl355_status_no_i2c_acknowledge;
  if (i2c_status != i2c_status_ok)
    return adxl355_status_i2c_transfer_error;

  *value = buffer;

  return adxl355_status_ok;
}


/*********************************************************************
   \brief Write register value

   \param[in]  uint8_t  : register address.
   \param[in]  uint8_t  : register value.

   \return adxl355_status : status of adxl355
         - adxl355_status_ok : I2C transfer completed successfully
         - adxl355_status_i2c_transfer_error : Problem with i2c transfer
         - adxl355_status_no_i2c_acknowledge : I2C did not acknowledge
*/
enum adxl355_status adxl355::write_user_register(uint8_t reg, uint8_t value) {
  enum adxl355_status status;
  uint8_t i2c_status;

  // Send the read command
  Wire.beginTransmission((uint8_t)ADXL355_ADDR);
  Wire.write((uint8_t)reg);
  Wire.write((uint8_t)value);
  i2c_status = Wire.endTransmission();

  if (i2c_status == i2c_status_err_overflow)
    return adxl355_status_no_i2c_acknowledge;
  if (i2c_status != i2c_status_ok)
    return adxl355_status_i2c_transfer_error;

  return adxl355_status_ok;
}
