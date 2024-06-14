#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#define COEFFICIENT_NUMBERS 7

enum adxl355_humidity_i2c_master_mode { adxl355_i2c_hold, adxl355_i2c_no_hold };

enum adxl355_status {
  adxl355_status_ok,
  adxl355_status_no_i2c_acknowledge,
  adxl355_status_i2c_transfer_error,
  adxl355_status_crc_error,
  adxl355_status_fifo_error
};

enum adxl355_humidity_resolution {
  adxl355_humidity_resolution_12b = 0,
  adxl355_humidity_resolution_11b,
  adxl355_humidity_resolution_10b,
  adxl355_humidity_resolution_8b
};

enum adxl355_battery_status { adxl355_battery_ok, adxl355_battery_low };

enum adxl355_heater_status { adxl355_heater_off, adxl355_heater_on };

enum adxl355_pressure_resolution {
  adxl355_pressure_resolution_osr_256 = 0,
  adxl355_pressure_resolution_osr_512,
  adxl355_pressure_resolution_osr_1024,
  adxl355_pressure_resolution_osr_2048,
  adxl355_pressure_resolution_osr_4096,
  adxl355_pressure_resolution_osr_8192
};

enum i2c_status_code {
  i2c_status_ok = 0x00,
  i2c_status_err_overflow = 0x01,
  i2c_status_err_timeout = 0x02,
};

class adxl355 {

  public:
    //adxl355();

    /****************************************************************
       \brief Perform initial configuration. Has to be called once.
    */
    void begin();

    /****************************************************************
      \brief Check whether adxl355 device is connected

      \return bool : status of adxl355
            - true : Device is present
            - false : Device is not acknowledging I2C address
    */
    bool is_connected(void);

    /****************************************************************
       \brief Reset the adxl355 device

       \return adxl355_status : status of adxl355
             - adxl355_status_ok : I2C transfer completed successfully
             - adxl355_status_i2c_transfer_error : Problem with i2c transfer
             - adxl355_status_no_i2c_acknowledge : I2C did not acknowledge
    */
    enum adxl355_status reset(void);

    /****************************************************************
       \brief Reads the temperature and pressure value.

       \param[out] float* : degC temperature value
       \param[out] float* : mbar pressure value

       \return adxl355_status : status of adxl355
             - adxl355_status_ok : I2C transfer completed successfully
             - adxl355_status_i2c_transfer_error : Problem with i2c transfer
             - adxl355_status_no_i2c_acknowledge : I2C did not acknowledge
             - adxl355_status_crc_error : CRC check error
    */
    enum adxl355_status read_temperature(float *t);

    /****************************************************************
       \brief Reads the fifo entries.

       \return int numbers of entries (0 to 127)
    */
    int fifo_entries();

    /****************************************************************
       \brief Reads the fifo samples.

       \return int numbers of samples
    */
    int read_fifo_samples();

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
    enum adxl355_status read_acceleration(float *x, float *y, float *z);


    /****************************************************************
        \brief Reads the acceleration in fifo register.

        \param[in] float* : pointer to acc vector

        \return adxl355_status : status of adxl355
              - adxl355_status_ok : I2C transfer completed successfully
              - adxl355_status_i2c_transfer_error : Problem with i2c transfer
              - adxl355_status_no_i2c_acknowledge : I2C did not acknowledge
              - adxl355_status_crc_error : CRC check error
    */
    enum adxl355_status read_acceleration_array(float *p);


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
    enum adxl355_status read_acceleration_array_byte(uint8_t *p, int *number_bytes, uint8_t *num_entries);


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
    enum adxl355_status read_acceleration_array_byte_test(uint8_t *p, int *number_bytes, uint8_t *num_entries, int count);


    /****************************************************************
       \brief Reads the acceleration X.

       \param[out] float* : acceleration x

       \return adxl355_status : status of adxl355
             - adxl355_status_ok : I2C transfer completed successfully
             - adxl355_status_i2c_transfer_error : Problem with i2c transfer
             - adxl355_status_no_i2c_acknowledge : I2C did not acknowledge
             - adxl355_status_crc_error : CRC check error
    */
    enum adxl355_status read_acc_X(float *x);


    /****************************************************************
       \brief Reads the acceleration Y.

       \param[out] float* : acceleration y

       \return adxl355_status : status of adxl355
             - adxl355_status_ok : I2C transfer completed successfully
             - adxl355_status_i2c_transfer_error : Problem with i2c transfer
             - adxl355_status_no_i2c_acknowledge : I2C did not acknowledge
             - adxl355_status_crc_error : CRC check error
    */
    enum adxl355_status read_acc_Y(float *y);


    /****************************************************************
       \brief Reads the acceleration Z.

       \param[out] float* : acceleration z

       \return adxl355_status : status of adxl355
             - adxl355_status_ok : I2C transfer completed successfully
             - adxl355_status_i2c_transfer_error : Problem with i2c transfer
             - adxl355_status_no_i2c_acknowledge : I2C did not acknowledge
             - adxl355_status_crc_error : CRC check error
    */
    enum adxl355_status read_acc_Z(float *z);


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
    enum adxl355_status config_filter(uint8_t hpf, uint8_t lpf);

    /****************************************************************
        \brief Config FIFO samples.

        \param[in] uint8_t* : fifo sample size

        \return adxl355_status : status of adxl355
              - adxl355_status_ok : I2C transfer completed successfully
              - adxl355_status_i2c_transfer_error : Problem with i2c transfer
              - adxl355_status_no_i2c_acknowledge : I2C did not acknowledge
              - adxl355_status_crc_error : CRC check error
    */
    enum adxl355_status config_fifo_samples(uint8_t samples);

    /****************************************************************
           \brief Turn on acc sensor.

           \return adxl355_status : status of adxl355
                 - adxl355_status_ok : I2C transfer completed successfully
                 - adxl355_status_i2c_transfer_error : Problem with i2c transfer
                 - adxl355_status_no_i2c_acknowledge : I2C did not acknowledge
                 - adxl355_status_crc_error : CRC check error
    */
    enum adxl355_status sensor_on();

    /****************************************************************
       \brief Turn off acc sensor (stand by)

       \return adxl355_status : status of adxl355
             - adxl355_status_ok : I2C transfer completed successfully
             - adxl355_status_i2c_transfer_error : Problem with i2c transfer
             - adxl355_status_no_i2c_acknowledge : I2C did not acknowledge
             - adxl355_status_crc_error : CRC check error
    */
    enum adxl355_status sensor_off();

    /****************************************************************
       \brief Reads the adxl355 user register.

       \param[in]  uint8_t  : register address.
       \param[in]  uint8_t  : number of bytes to be read.
       \param[out] uint8_t* : register value.

       \return adxl355_status : status of adxl355
             - adxl355_status_ok : I2C transfer completed successfully
             - adxl355_status_i2c_transfer_error : Problem with i2c transfer
             - adxl355_status_no_i2c_acknowledge : I2C did not acknowledge
    */
    enum adxl355_status read_user_register(uint8_t reg, uint8_t *value);

    /****************************************************************
       \brief Writes the adxl355 user register with value

       \param[in]  uint8_t  : register address.
       \param[in]  uint8_t  : register value.

       \return adxl355_status : status of adxl355
             - adxl355_status_ok : I2C transfer completed successfully
             - adxl355_status_i2c_transfer_error : Problem with i2c transfer
             - adxl355_status_no_i2c_acknowledge : I2C did not acknowledge
    */
    enum adxl355_status write_user_register(uint8_t reg, uint8_t value);



    /************************ PRIVATE ***********************************/

  private:
    /* Variables */
    int32_t memory_dT;



};
